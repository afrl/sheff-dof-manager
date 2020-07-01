#include "dof_manager/detail/EnumerationAlgorithms.h"

#include "dof_manager/DofRanges.h"
#include "dof_manager/ElementScatterMap.h"
#include "dof_manager/NodeScatterMap.h"

#include "utility/MPIUtils.h"

#include <fstream>

namespace dof_manager
{
    namespace detail
    {
        void create_initial_enumeration(
            const MeshType& mesh,
            const ElementScatterMap& element_scatter_map,
            DofMap& dof_map)
        {
            auto local_rank = utility::MPIUtils::get_local_rank();
            // This is failing right now.  The partitioning algorithm is wrong
            assert(mesh.elements()[0]->get_parent_partition() == local_rank);

            // For now, assume each node has the same number of DoF's at the start of the analysis.
            // This algorithm will have to be generalized if this is not the case.
            auto initial_dofs_per_node = mesh.elements().front()->get_n_dofs_per_point();

            dof_map.element_to_rank_maps.resize(mesh.elements().size(), nullptr);
            dof_map.element_n_dofs.resize(mesh.elements().size(), 0);

            // At this point, nodes only have one set of DoF's associated with them.  For the set of
            // nodes that appear in the connectivity of owned elements, count the number of DoF's on
            // this rank in each interval.
            dof_map.n_owned_dofs = 0;
            for (auto& n_ptr : mesh.nodes())
            {
                if (n_ptr->get_parent_partition() == local_rank)
                {
                    dof_map.n_owned_dofs += initial_dofs_per_node;
                }
            }
            auto n_all_dofs_on_rank = initial_dofs_per_node * mesh.nodes().size();

            auto n_owned_dofs_by_rank = utility::MPIUtils::all_gather_vectors<GlobalDofIndex>(
                { static_cast<GlobalDofIndex>(dof_map.n_owned_dofs) });

            // Determine the range of dof's this rank owns
            // We assume that each rank will own a contiguous range, so we can determined
            // the owned range of this rank based on the n_owned_dofs in each rank.
            dof_map.owned_global_dof_begin = std::accumulate(
                n_owned_dofs_by_rank.begin(),
                n_owned_dofs_by_rank.begin() + local_rank,
                0);
            dof_map.owned_global_dof_end = dof_map.owned_global_dof_begin + dof_map.n_owned_dofs - 1;

            // Create the initial rank-to-global map that follows the ordering of the local indices
            // of the nodes.  The intervals will be restored later.
            dof_map.rank_to_global_map.resize(
                n_all_dofs_on_rank,
                std::numeric_limits<GlobalDofIndex>::max());
            GlobalDofIndex curr_global_index = dof_map.owned_global_dof_begin;
            std::vector<GlobalDofIndex> node_to_dof_offset_map(
                mesh.nodes().size(),
                std::numeric_limits<GlobalDofIndex>::max());
            for (auto& n_ptr : mesh.nodes())
            {
                if (n_ptr->get_parent_partition() == local_rank)
                {
                    node_to_dof_offset_map[n_ptr->get_local_number()] = curr_global_index;
                    for (auto component : utility::range_from_zero(initial_dofs_per_node))
                    {
                        const auto rank_index = initial_dofs_per_node * n_ptr->get_local_number() + component;
                        const auto global_index = curr_global_index++;
                        dof_map.rank_to_global_map[rank_index] = global_index;
                    }
                }
            }

            // The owned nodes were enumerated, but a step of MPI communication is needed to
            // determine the global DoF's of the ghost nodes.
            if (utility::MPIUtils::is_distributed())
            {
                // Right now, this is the only part of the code that requires a NodeScatterMap.  If the analysis or
                // another persistent object requires a NodeScatterMap, then it should be passed into this method
                // instead of temporarily created.
                NodeScatterMap node_scatter_map(mesh, element_scatter_map);

                // Populate the a list of global DoF offsets (the first DoF of the node) for ghost nodes in the
                // other ranks
                std::vector<GlobalDofIndex> send_buffer;
                for (auto dest : node_scatter_map.destination_ranks_range())
                {
                    for (auto node_local_id : node_scatter_map.nodes_to_send_range(dest))
                    {
                        const auto rank_dof_offset = initial_dofs_per_node * node_local_id;
                        const auto global_dof_offset = dof_map.rank_to_global_map[rank_dof_offset];
                        send_buffer.push_back(global_dof_offset);
                    }
                }

                // Exchange the DoF offsets with ghost layers
                std::vector<GlobalDofIndex> receive_buffer;
                std::vector<size_t> receive_offsets;
                utility::MPIUtils::exchange_arrays<GlobalDofIndex, size_t>(
                    static_cast<int>(node_scatter_map.destination_ranks.size()),
                    node_scatter_map.destination_ranks.data(),
                    send_buffer.data(),
                    node_scatter_map.rank_send_offsets.data(),
                    static_cast<int>(node_scatter_map.source_ranks.size()),
                    node_scatter_map.source_ranks.data(),
                    receive_buffer,
                    receive_offsets);

                // Populate the dof_map.rank_to_global_map for DoF's correspond to ghost nodes,
                // which was just communicated above.
                size_t offset = 0;
                for (auto source : node_scatter_map.source_ranks_range())
                {
                    for (auto node_local_id : node_scatter_map.nodes_to_receive_range(source))
                    {
                        const auto& node_global_dof_offset = receive_buffer[offset++];
                        
                        for (auto component : utility::range_from_zero(initial_dofs_per_node))
                        {
                            const auto rank_index = initial_dofs_per_node * node_local_id + component;
                            const auto global_index = node_global_dof_offset + component;
                            dof_map.rank_to_global_map[rank_index] = global_index;
                        }
                    }
                }
            }

            assert(std::none_of(
                dof_map.rank_to_global_map.begin(),
                dof_map.rank_to_global_map.end(),
                [](const GlobalDofIndex& dof) -> bool {
                    return dof == std::numeric_limits<GlobalDofIndex>::max();
                }));

            // Create the initial element-rank-map
            dof_map.n_owned_elements = 0;
            for (auto& e_ptr : mesh.elements())
            {
                const ElementDofIndex n_dofs_for_element =
                    static_cast<ElementDofIndex>(e_ptr->get_num_nodes() * initial_dofs_per_node);
                dof_map.element_to_rank_maps[e_ptr->get_local_number()] =
                    new RankDofIndex[n_dofs_for_element];
                dof_map.element_n_dofs[e_ptr->get_local_number()] = n_dofs_for_element;

                auto& element_dof_map = dof_map.element_to_rank_maps[e_ptr->get_local_number()];
                ElementDofIndex node_index = 0;
                for (auto& n_ptr : *e_ptr)
                {
                    for (auto component : utility::range_from_zero(initial_dofs_per_node))
                    {
                        element_dof_map[node_index * initial_dofs_per_node + component] =
                            initial_dofs_per_node * n_ptr->get_local_number() + component;
                    }
                    node_index++;
                }

                if (e_ptr->get_parent_partition() == local_rank)
                {
                    dof_map.n_owned_elements++;
                }
            }

            /*
            std::ofstream dof_out("dof_map_before_sort_" + std::to_string(utility::MPIUtils::get_local_rank()) + ".txt");
            dof_out << dof_map.str(mesh);
            dof_out.close();
            */

            sort_intervals(dof_map);
        }


        UpdateMapsLocalEnrichment create_maps_for_local_enrichment_and_combinations(
            const DofMap& dof_map,
            EnrichmentPhaseData& enrichment_data)
        {
            auto& new_dof_to_old_rank_map = enrichment_data.new_dof_to_old_rank_map;

            UpdateMapsLocalEnrichment update_maps;

            // Filter out any pair of DoFs that should be combined that are both owned
            // (removes any owned pairs from enrichment_data.dof_to_combine_map, which will
            // be combined in a later step).
            auto& owned_dofs_to_combine = update_maps.owned_dofs_to_combine;
            for (auto iter = enrichment_data.dof_to_combine_map.begin();
                iter != enrichment_data.dof_to_combine_map.end();)
            {
                if (dof_map.get_dof_type(iter->first) == DofType::OWNED &&
                    dof_map.get_dof_type(iter->second) == DofType::OWNED)
                {
                    owned_dofs_to_combine.insert(*iter);
                    enrichment_data.dof_to_combine_map.erase(iter++);
                }
                else
                {
                    iter++;
                }
            }

            // Find how many new DoF's were created on each rank
            const auto n_new_dofs = static_cast<int32_t>(new_dof_to_old_rank_map.size());
            const auto n_dofs_to_eliminate = static_cast<int32_t>(owned_dofs_to_combine.size());
            const int32_t delta_n_dofs_for_this_rank = n_new_dofs - n_dofs_to_eliminate;

            update_maps.n_new_dofs_on_this_rank = n_new_dofs;

            // The new rank numbering will restore the [ owned, local ghosts, exclusive ghosts ]
            // intervals of the rank numbering and resolve any pairs of DoF's to combine for which
            // both are owned.  Combinations involving ghosts are resolved in a later step.
            //
            // The post-enrichment numbering has the new DoF's added during the enrichment phase
            // by this rank (no new ghosts), which were added to the end of the range.
            auto& post_enrichment_rank_to_new_rank_map = update_maps.post_enrichment_rank_to_new_rank_map;
            // The pre-enrichment numbering does not have any new DoF's from the enrichment phase.
            // This map points to the DoF that either corresponded to the new DoF or was cloned for
            // the new DoF.
            auto& new_rank_to_pre_enrichment_rank_map = update_maps.new_rank_to_pre_enrichment_rank_map;
            post_enrichment_rank_to_new_rank_map.resize(dof_map.n_dofs_on_rank() + n_new_dofs);
            new_rank_to_pre_enrichment_rank_map.resize(dof_map.n_dofs_on_rank() + delta_n_dofs_for_this_rank);

            // Create the new rank to old rank map and the old rank to new rank map
            RankDofIndex new_rank_index = 0;
            // Update owned DoF's
            for (auto owned_dof : OwnedRange(dof_map))
            {
                // Performance note: if we use a map instead of unordered_map, we could increment
                // the iterator as we loop since both would ordered.  Eliminating the need to
                // search with the .count() calls.
                if (owned_dofs_to_combine.count(owned_dof.rank_dof) == 0)
                {
                    new_rank_to_pre_enrichment_rank_map[new_rank_index] = owned_dof.rank_dof;
                    post_enrichment_rank_to_new_rank_map[owned_dof.rank_dof] = new_rank_index++;
                }
                //else
                //{
                    // post_enrichment_rank_to_new_rank_map[owned_dof.rank_dof] needs to be set
                    // to the new rank index of find_iter after the loops
                    // (See loop over owned_dofs_to_combine)
                //}
            }
            // Insert the new DoF's at the end of the owned range
            for (auto i : utility::range_from_zero(new_dof_to_old_rank_map.size()))
            {
                // Note: before this renumbering new DoFs would have been added to the end
                // of the range, after all ghosts, so the rank number after enrichment is:
                const auto post_enrch_rank_number = static_cast<GlobalDofIndex>(dof_map.n_dofs_on_rank() + i);
                if (owned_dofs_to_combine.count(post_enrch_rank_number) == 0)
                {
                    new_rank_to_pre_enrichment_rank_map[new_rank_index] = new_dof_to_old_rank_map[i];
                    post_enrichment_rank_to_new_rank_map[post_enrch_rank_number] = new_rank_index++;
                }
                //else
                //{
                    // post_enrichment_rank_to_new_rank_map[n_dofs_on_rank() + i] needs to be set
                    // to the new rank index of find_iter after the loops
                    // (See loop over owned_dofs_to_combine)
                //}
            }
            // Update local ghost interval
            for (auto ghost : GhostRange(dof_map, GhostRangeType::LOCAL))
            {
                new_rank_to_pre_enrichment_rank_map[new_rank_index] = ghost.rank_dof;
                post_enrichment_rank_to_new_rank_map[ghost.rank_dof] = new_rank_index++;
            }
            // Update all exclusive ghost DoF's
            for (auto ghost : GhostRange(dof_map, GhostRangeType::EXCLUSIVE))
            {
                new_rank_to_pre_enrichment_rank_map[new_rank_index] = ghost.rank_dof;
                post_enrichment_rank_to_new_rank_map[ghost.rank_dof] = new_rank_index++;
            }

            // Complete the map for the eliminated DoF's that were combined into another owned DoF.
            // Also, track which DoF's were combined into with the new rank numbering in order to
            // determine if a DoF neds to be communciated in exchange_new_exclusive_ghost_dofs().
            for (auto p : owned_dofs_to_combine)
            {
                post_enrichment_rank_to_new_rank_map[p.first] =
                    post_enrichment_rank_to_new_rank_map[p.second];

                update_maps.owned_post_enrichment_dofs_combined_into.insert(p.second);
            }

            // Communicate how many DoF's were created/eliminated on each rank
            update_maps.delta_n_dofs_by_rank =
                utility::MPIUtils::all_gather_vectors<int32_t>({ delta_n_dofs_for_this_rank });

            return update_maps;
        }

        RenumberingPhaseData update_owned_global_numbering(
            const UpdateMapsLocalEnrichment& update_maps, 
            DofMap& dof_map)
        {
            const auto n_new_dofs_on_this_rank = update_maps.n_new_dofs_on_this_rank;
            const auto delta_n_dofs_for_this_rank =
                update_maps.delta_n_dofs_by_rank[utility::MPIUtils::get_local_rank()];
            RankDofIndex delta_n_dofs_from_lower_ranks = std::accumulate(
                update_maps.delta_n_dofs_by_rank.begin(),
                update_maps.delta_n_dofs_by_rank.begin() + utility::MPIUtils::get_local_rank(),
                0);

            RenumberingPhaseData renumber_data;
            renumber_data.old_rank_to_global_map.swap(dof_map.rank_to_global_map);
            renumber_data.pre_enrichment_n_owned_dofs = dof_map.n_owned_dofs;

            dof_map.n_owned_dofs += delta_n_dofs_for_this_rank;
            dof_map.owned_global_dof_begin += delta_n_dofs_from_lower_ranks;
            dof_map.owned_global_dof_end += delta_n_dofs_from_lower_ranks + delta_n_dofs_for_this_rank;

            // Resize the rank-to-global map to only hold the owned DoF's for now.  The ghosts will
            // be added when the ghost layer is updated.
            dof_map.rank_to_global_map.resize(dof_map.n_owned_dofs);
            GlobalDofIndex curr_global_index = dof_map.owned_global_dof_begin;
            for (auto i : utility::range_from_zero(dof_map.n_owned_dofs))
            {
                dof_map.rank_to_global_map[i] = curr_global_index++;
            }
            assert(dof_map.owned_global_dof_end == curr_global_index - 1);

            // Note: renumber_data.owned_renumbering_sources will be modified later 
            // to account for inter-rank DoF combinations
            renumber_data.owned_renumbering_sources.reserve(dof_map.n_owned_dofs);
            for (auto i : utility::range_from_zero(dof_map.n_owned_dofs))
            {
                renumber_data.owned_renumbering_sources.push_back({
                        DofRenumberingSource::COPY_OLD_LOCAL_VALUE,
                        update_maps.new_rank_to_pre_enrichment_rank_map[i]
                    });
            }

            return renumber_data;
        }

        void update_ghost_pre_enrichment_global_numbering(
            const ElementScatterMap& element_scatter, 
            const RenumberingPhaseData& renumbering_data, 
            UpdateMapsLocalEnrichment& update_maps,
            DofMap& dof_map)
        {
            // Note: the element-to-rank maps have NOT been updated to new rank numbering, so they
            //       still follow the post-enrichment numbering.  This is intenetional as it simplifies
            //       the algorithm.
            // For each rank that contains DoF's owned by this rank as ghosts, create a map from
            // the old global index to the new global index for any of the existing ghost DoF's, excludes
            // new DoF's created during enrichment.
            const auto& pre_enrichment_n_owned_dofs = renumbering_data.pre_enrichment_n_owned_dofs;
            std::vector<GlobalDofIndex> send_buffer;
            std::vector<size_t> send_offsets = { 0 };
            for (auto i : utility::range_from_zero(element_scatter.destination_ranks.size()))
            {
                auto& rank = element_scatter.destination_ranks[i];

                std::map<GlobalDofIndex, GlobalDofIndex> existing_global_to_new_global_map;

                for (auto offset = element_scatter.rank_send_offsets[i];
                    offset < element_scatter.rank_send_offsets[i + 1];
                    offset++)
                {
                    auto element_index = element_scatter.elements_to_send[offset];

                    for (auto j : utility::range_from_zero(dof_map.element_n_dofs[element_index]))
                    {
                        auto& post_enrichment_rank_index = dof_map.element_to_rank_maps[element_index][j];

                        // This DoF existed before enrichment (pre-enrichment) and is owned
                        if (post_enrichment_rank_index < pre_enrichment_n_owned_dofs)
                        {
                            auto& new_rank_index = update_maps.post_enrichment_rank_to_new_rank_map[post_enrichment_rank_index];
                            auto& new_global_index = dof_map.rank_to_global_map[new_rank_index];
                            auto& old_global_index = renumbering_data.old_rank_to_global_map[post_enrichment_rank_index];
                            existing_global_to_new_global_map[old_global_index] = new_global_index;
                        }
                    }
                }

                // Add the map to the send buffer based on existing_global_to_new_global_map
                send_buffer.reserve(send_buffer.size() + 2 * existing_global_to_new_global_map.size());
                for (auto& p : existing_global_to_new_global_map)
                {
                    send_buffer.push_back(p.first);
                    send_buffer.push_back(p.second);
                }
                send_offsets.push_back(send_buffer.size());
            }
            send_offsets.push_back(send_buffer.size());

            // Exchange data with ghost layers
            std::vector<GlobalDofIndex> receive_buffer;
            std::vector<size_t> receive_offsets;
            utility::MPIUtils::exchange_arrays<GlobalDofIndex, size_t>(
                static_cast<int>(element_scatter.destination_ranks.size()),
                element_scatter.destination_ranks.data(),
                send_buffer.data(),
                send_offsets.data(),
                static_cast<int>(element_scatter.source_ranks.size()),
                element_scatter.source_ranks.data(),
                receive_buffer,
                receive_offsets);

            // This will require a ghost old global -> rank map
            std::map<GlobalDofIndex, RankDofIndex> ghost_pre_enrichment_global_to_new_rank_map;
            for (auto i : utility::range<RankDofIndex>(renumbering_data.pre_enrichment_n_owned_dofs,
                static_cast<RankDofIndex>(renumbering_data.old_rank_to_global_map.size())))
            {
                ghost_pre_enrichment_global_to_new_rank_map[renumbering_data.old_rank_to_global_map[i]] =
                    update_maps.post_enrichment_rank_to_new_rank_map[i];
            }

            // Decode the received maps and update the rank-to-global map for any of the ghost
            // DoF's that previously existed (excludes new DoF's on other ranks).
            const auto n_new_dofs = receive_buffer.size() / 2;
            dof_map.rank_to_global_map.resize(dof_map.rank_to_global_map.size() + n_new_dofs);
            for (auto i : utility::range_from_zero(element_scatter.source_ranks.size()))
            {
                auto& source_rank = element_scatter.source_ranks[i];
                auto curr_offset = receive_offsets[i];
                while (curr_offset != receive_offsets[i + 1])
                {
                    assert(curr_offset < receive_offsets[i + 1]);

                    auto& pre_enrichment_global_index = receive_buffer[curr_offset++];
                    auto& new_global_index = receive_buffer[curr_offset++];

                    auto& new_rank_index = ghost_pre_enrichment_global_to_new_rank_map[pre_enrichment_global_index];
                    dof_map.rank_to_global_map[new_rank_index] = new_global_index;
                }
            }
        }

        void exchange_new_exclusive_ghost_dofs(
            const ElementScatterMap& element_scatter, 
            const RenumberingPhaseData& renumbering_data, 
            const UpdateMapsLocalEnrichment& update_maps, 
            DofMap& dof_map)
        {
            // Now the new ghost DoF's can be communicated
            const auto pre_enrichment_n_dofs = renumbering_data.old_rank_to_global_map.size();
            std::vector<GlobalDofIndex> send_buffer;
            std::vector<size_t> send_offsets = { 0 };
            for (auto i : utility::range_from_zero(element_scatter.destination_ranks.size()))
            {
                auto& rank = element_scatter.destination_ranks[i];

                std::map<GlobalDofIndex, GlobalDofIndex> new_dof_global_to_pre_enrichment_global_map;

                for (auto offset = element_scatter.rank_send_offsets[i];
                    offset < element_scatter.rank_send_offsets[i + 1];
                    offset++)
                {
                    auto element_index = element_scatter.elements_to_send[offset];

                    for (auto j : utility::range_from_zero(dof_map.element_n_dofs[element_index]))
                    {
                        auto& post_enrichment_rank_index = dof_map.element_to_rank_maps[element_index][j];

                        // This DoF existed before enrichment (pre-enrichment) and is owned
                        if (post_enrichment_rank_index >= pre_enrichment_n_dofs)
                        {
                            // Make sure this was not a DoF being combined into by another DoF that
                            // already existed, since it would have been communicated earlier in
                            // update_ghost_pre_enrichment_global_numbering()
                            if (update_maps.owned_post_enrichment_dofs_combined_into.count(post_enrichment_rank_index) == 0)
                            {
                                auto& new_rank_index = update_maps.post_enrichment_rank_to_new_rank_map[post_enrichment_rank_index];
                                auto& new_global_index = dof_map.rank_to_global_map[new_rank_index];
                                auto& source_pre_enrichment_rank_index = update_maps.new_rank_to_pre_enrichment_rank_map[new_rank_index];
                                auto& source_pre_enrichment_global_index = renumbering_data.old_rank_to_global_map[source_pre_enrichment_rank_index];
                                new_dof_global_to_pre_enrichment_global_map[new_global_index] = source_pre_enrichment_global_index;
                            }
                        }
                    }
                }

                // Add the map to the send buffer based on existing_global_to_new_global_map
                send_buffer.reserve(send_buffer.size() +
                    2 * new_dof_global_to_pre_enrichment_global_map.size());
                for (auto& p : new_dof_global_to_pre_enrichment_global_map)
                {
                    send_buffer.push_back(p.first);
                    send_buffer.push_back(p.second);
                }
                send_offsets.push_back(send_buffer.size());
            }
            send_offsets.push_back(send_buffer.size());

            // Exchange data with ghost layers
            std::vector<GlobalDofIndex> receive_buffer;
            std::vector<size_t> receive_offsets;
            utility::MPIUtils::exchange_arrays<GlobalDofIndex, size_t>(
                static_cast<int>(element_scatter.destination_ranks.size()),
                element_scatter.destination_ranks.data(),
                send_buffer.data(),
                send_offsets.data(),
                static_cast<int>(element_scatter.source_ranks.size()),
                element_scatter.source_ranks.data(),
                receive_buffer,
                receive_offsets);

            // Decode the received maps and add new ghost DoF's to the rank-to-global map
            // in the exclusive ghost interval.
            for (auto i : utility::range_from_zero(element_scatter.source_ranks.size()))
            {
                auto& source_rank = element_scatter.source_ranks[i];
                auto curr_offset = receive_offsets[i];
                while (curr_offset != receive_offsets[i + 1])
                {
                    assert(curr_offset < receive_offsets[i + 1]);

                    auto& new_global_index = receive_buffer[curr_offset++];
                    auto& source_pre_enrichment_global_index = receive_buffer[curr_offset++];

                    dof_map.rank_to_global_map.push_back(new_global_index);

                    // Note: right now, source_pre_enrichment_global_index is not being used.
                    //       Does this need to be tracked for some situations with combinations?  If
                    //       so, then we need to update update_maps.new_rank_to_pre_enrichment_rank_map
                }
            }
        }

        void update_element_to_rank_map(
            const UpdateMapsLocalEnrichment& update_maps, 
            DofMap& dof_map)
        {
            for (auto local_element_number : utility::range_from_zero(dof_map.element_to_rank_maps.size()))
            {
                auto& element_to_rank_map = dof_map.element_to_rank_maps[local_element_number];
                for (auto i : utility::range_from_zero(dof_map.element_n_dofs[local_element_number]))
                {
                    const auto old_rank_index = element_to_rank_map[i];
                    element_to_rank_map[i] =
                        update_maps.post_enrichment_rank_to_new_rank_map[old_rank_index];
                }
            }
        }

        std::unordered_map<GlobalDofIndex, RankDofIndex> get_global_to_rank_map(
            const DofMap& dof_map)
        {
            // The global-to-ghost map is needed to know if one of the received DoF's is a
            // new DoF and to map the global DoF's in the receive buffer to rank numbering.
            std::unordered_map<GlobalDofIndex, RankDofIndex> global_to_rank_map;
            const auto n_dofs_on_rank = static_cast<RankDofIndex>(dof_map.rank_to_global_map.size());
            for (auto i : utility::range_from_zero(n_dofs_on_rank))
            {
                global_to_rank_map[dof_map.rank_to_global_map[i]] = i;
            }
            return global_to_rank_map;
        }

        void exchange_enriched_ghost_elements(
            const EnrichmentPhaseData& enrichment_data, 
            const ElementScatterMap& element_scatter, 
            const std::unordered_map<GlobalDofIndex, RankDofIndex>& global_to_rank_map,
            DofMap& dof_map)
        {
            // Make sure we won't overflow
            assert(sizeof(GlobalDofIndex) >= sizeof(ElementLocalIndex));

            const auto n_new_dofs_on_this_rank = enrichment_data.new_dof_to_old_rank_map.size();

            // Create send buffers
            // The send buffer will contain the element to global DoF maps of any enriched element.
            // The buffer is arranged following the element DofManager, so there may be duplicates
            // if an element needs to be sent to multiple ranks.  For each entry, the buffer contains:
            // (true, # DoF's for element, global DoF 1, global DoF 1 that was cloned, ...)
            // for elements in element_scatter.elements_to_send that are modified, and:
            // (false)
            // for elements that were not modified.
            std::vector<GlobalDofIndex> send_buffer;
            std::vector<size_t> send_offsets = { 0 };
            for (auto dest_rank : element_scatter.destination_ranks_range())
            {
                for (auto local_element_index : element_scatter.elements_to_send_range(dest_rank))
                {
                    if (enrichment_data.element_enriched[local_element_index])
                    {
                        send_buffer.push_back(1);
                        send_buffer.push_back(dof_map.element_n_dofs[local_element_index]);
                        for (auto j : utility::range_from_zero(dof_map.element_n_dofs[local_element_index]))
                        {
                            auto& rank_index = dof_map.element_to_rank_maps[local_element_index][j];
                            send_buffer.push_back(dof_map.rank_to_global_map[rank_index]);
                        }
                    }
                    else
                    {
                        send_buffer.push_back(0);
                    }
                }
                send_offsets.push_back(send_buffer.size());
            }
            send_offsets.push_back(send_buffer.size());

            // Exchange data with ghost layers
            std::vector<GlobalDofIndex> receive_buffer;
            std::vector<size_t> receive_offsets;
            utility::MPIUtils::exchange_arrays<GlobalDofIndex, size_t>(
                static_cast<int>(element_scatter.destination_ranks.size()),
                element_scatter.destination_ranks.data(),
                send_buffer.data(),
                send_offsets.data(),
                static_cast<int>(element_scatter.source_ranks.size()),
                element_scatter.source_ranks.data(),
                receive_buffer,
                receive_offsets);

            // Update the element to rank DoF maps for the received ghost elements, adding
            // new DoF's to the rank map.
            for (auto source_rank : element_scatter.source_ranks_range())
            {
                auto curr_offset = receive_offsets[source_rank.index];
                for (auto local_element_index : element_scatter.elements_to_receive_range(source_rank))
                {
                    assert(curr_offset < receive_offsets[source_rank.index + 1]);

                    auto is_modified = static_cast<bool>(receive_buffer[curr_offset++]);

                    if (is_modified)
                    {
                        auto element_n_dofs = static_cast<RankDofIndex>(receive_buffer[curr_offset++]);

                        auto& element_to_rank_map = dof_map.element_to_rank_maps[local_element_index];
                        delete[] element_to_rank_map;
                        element_to_rank_map = new RankDofIndex[element_n_dofs];
                        dof_map.element_n_dofs[local_element_index] = element_n_dofs;

                        for (auto j : utility::range_from_zero(element_n_dofs))
                        {
                            auto global_dof = receive_buffer[curr_offset++];

                            auto iter = global_to_rank_map.find(global_dof);
                            assert(iter != global_to_rank_map.end());

                            const auto& rank_dof = iter->second;
                            element_to_rank_map[j] = rank_dof;
                        }
                    }

                }
                assert(curr_offset == receive_offsets[source_rank.index + 1]);
            }
        }

        void resolve_inter_rank_combinations(
            const EnrichmentPhaseData& enrichment_data, 
            const std::unordered_map<GlobalDofIndex, RankDofIndex>& global_to_rank_map, 
            const UpdateMapsLocalEnrichment& update_maps, 
            RenumberingPhaseData& renumbering_data,
            DofMap& dof_map)
        {
            // Create a list of the combinations as recorded by this rank as a single array.
            // This contains the [ global_dof_to_combine_lhs_1, global_dof_to_combine_rhs_1, ... ]
            std::vector<GlobalDofIndex> global_dofs_to_combine_on_this_rank;
            global_dofs_to_combine_on_this_rank.reserve(2 * enrichment_data.dof_to_combine_map.size());
            for (auto& pair : enrichment_data.dof_to_combine_map)
            {
                auto& new_rank_index_1 = update_maps.post_enrichment_rank_to_new_rank_map[pair.first];
                auto& new_rank_index_2 = update_maps.post_enrichment_rank_to_new_rank_map[pair.second];
                global_dofs_to_combine_on_this_rank.push_back(dof_map.rank_to_global_map[new_rank_index_1]);
                global_dofs_to_combine_on_this_rank.push_back(dof_map.rank_to_global_map[new_rank_index_2]);
            }
            auto global_dofs_to_combine_on_all_ranks =
                utility::MPIUtils::all_gather_vectors<GlobalDofIndex>(
                    global_dofs_to_combine_on_this_rank);

            if (global_dofs_to_combine_on_all_ranks.empty())
            {
                return;
            }

            // Because multiple ranks could have had combinations involving the same DoF's,
            // process the combinations to result in all DoF's being combined into another forming
            // a unique set that also does not appear in the list of DoF's being combined into.
            std::map<GlobalDofIndex, GlobalDofIndex> global_dofs_to_combine_map;
            std::multimap<GlobalDofIndex, GlobalDofIndex> global_dofs_combined_into_map;
            for (auto i = 0; i < global_dofs_to_combine_on_all_ranks.size(); i += 2)
            {
                auto& lhs = global_dofs_to_combine_on_all_ranks[i];
                auto& rhs = global_dofs_to_combine_on_all_ranks[i + 1];
                process_dof_combination<GlobalDofIndex>(
                    lhs,
                    rhs,
                    global_dofs_to_combine_map,
                    global_dofs_combined_into_map);
            }
            global_dofs_combined_into_map.clear();

            // Now change DoF's being combined into DoF's that do not exist on this rank and
            // save a map of DoF's to be combined into other DoF's if both are on this rank in
            // terms of the rank numbering.
            std::map<RankDofIndex, RankDofIndex> dofs_to_combine_on_this_rank;
            GlobalDofIndex n_dofs_eliminated_on_lower_ranks = 0;
            GlobalDofIndex n_dofs_eliminated_on_this_rank = 0;
            for (auto& v : global_dofs_to_combine_map)
            {
                auto& dof_being_combined_global_index = v.first;
                auto& dof_being_combined_into_global_index = v.second;

                auto find_iter = global_to_rank_map.find(dof_being_combined_global_index);
                if (find_iter != global_to_rank_map.end())
                {
                    auto& dof_being_combined_rank_index = find_iter->second;

                    // The global dof being combined into the second of the pair exists
                    // on this this rank.
                    auto find_iter2 = global_to_rank_map.find(dof_being_combined_into_global_index);
                    if (find_iter2 != global_to_rank_map.end())
                    {
                        auto& dof_being_combined_into_rank_index = find_iter2->second;
                        // Both DoFs involved in the combination exist on this rank,
                        // so one needs to be eliminated out of the system.
                        dofs_to_combine_on_this_rank[dof_being_combined_rank_index] =
                            dof_being_combined_into_rank_index;
                    }
                    else
                    {
                        // Only the DoF being combined into another exists on this
                        // ranks, so we can just replace it with the second DoF
                        dof_map.rank_to_global_map[dof_being_combined_rank_index] =
                            dof_being_combined_into_global_index;
                    }
                }

                if (dof_being_combined_global_index < dof_map.owned_global_dof_begin)
                {
                    n_dofs_eliminated_on_lower_ranks++;
                }
                else if (dof_being_combined_global_index >= dof_map.owned_global_dof_begin &&
                    dof_being_combined_global_index <= dof_map.owned_global_dof_end)
                {
                    n_dofs_eliminated_on_this_rank++;
                }
            }

            // Eliminate the DoF's appearing in dofs_to_combine_on_this_rank from the rank numbering.
            // First, update the renumbering_data.owned_new_rank_to_old_rank_map for any owned DoF's
            // being combined into another.
            RankDofIndex n_owned_dofs_eliminated = 0;
            for (auto& p : dofs_to_combine_on_this_rank)
            {
                if (p.first < dof_map.n_owned_dofs)
                {
                    // The DoF being combined into should never be owned as it would have been taken
                    // care of earlier.
                    assert(p.second >= dof_map.n_owned_dofs);

                    // The DoF being combined is owned, so the renumbering_data needs to be modified
                    // to point to the old rank index of the DoF being combined into.
                    auto& source_value = renumbering_data.owned_renumbering_sources[p.first];
                    source_value.source_type = DofRenumberingSource::COPY_NEW_GHOST_VALUE;
                    source_value.source_dof_rank_index = p.second;
                    n_owned_dofs_eliminated++;
                }
            }
            // Second, create a map for the rank numbering that maps between the state before
            // processing the inter-rank combinations to the state after the combinations.
            // Note: depends on the keys of dofs_to_combine_on_this_rank being sorted.
            std::vector<RankDofIndex> pre_combine_to_post_combine_map;
            pre_combine_to_post_combine_map.resize(dof_map.rank_to_global_map.size());
            auto curr_dof_to_combine_iter = dofs_to_combine_on_this_rank.begin();
            RankDofIndex post_combine_rank_index = 0;
            for (auto pre_combine_rank_index : utility::range_from_zero(dof_map.rank_to_global_map.size()))
            {
                if (curr_dof_to_combine_iter != dofs_to_combine_on_this_rank.end() &&
                    pre_combine_rank_index == curr_dof_to_combine_iter->first)
                {
                    // Skip any rank DoF's that were combined into others, update these later.
                    curr_dof_to_combine_iter++;
                }
                else
                {
                    pre_combine_to_post_combine_map[pre_combine_rank_index] = post_combine_rank_index++;
                }
            }
            // Now update rank DoF's that were combined into others
            for (auto& p : dofs_to_combine_on_this_rank)
            {
                pre_combine_to_post_combine_map[p.first] = pre_combine_to_post_combine_map[p.second];
            }
            // Create the reverse post-combine -> pre-combine
            std::vector<RankDofIndex> post_combine_to_pre_combine_map;
            post_combine_to_pre_combine_map.resize(*std::max_element(
                pre_combine_to_post_combine_map.begin(),
                pre_combine_to_post_combine_map.end()) + 1);
            for (auto i : utility::indices(pre_combine_to_post_combine_map))
            {
                post_combine_to_pre_combine_map[pre_combine_to_post_combine_map[i]] = i;
            }
            // Since the pre-combine -> post-combine is not a unique relationship, correct
            // the entry for any combinations since they might have the wrong value
            for (auto& p : dofs_to_combine_on_this_rank)
            {
                auto post_combine_index = pre_combine_to_post_combine_map[p.first];
                post_combine_to_pre_combine_map[post_combine_index] = p.second;
            }
            // Third, update:
            //  - renumbering_data.owned_new_rank_to_old_rank_map
            //  - dof_map.rank_to_global_map
            //  - element_to_rank_maps_
            renumbering_data.permute_owned_renumbering_sources(
                static_cast<RankDofIndex>(dof_map.n_owned_dofs - n_owned_dofs_eliminated),
                post_combine_to_pre_combine_map,
                pre_combine_to_post_combine_map);
            apply_permutation(
                dof_map.rank_to_global_map,
                post_combine_to_pre_combine_map);
            for (auto local_element_number : utility::range_from_zero(dof_map.element_to_rank_maps.size()))
            {
                auto& element_to_rank_map = dof_map.element_to_rank_maps[local_element_number];
                for (auto i : utility::range_from_zero(dof_map.element_n_dofs[local_element_number]))
                {
                    element_to_rank_map[i] =
                        pre_combine_to_post_combine_map[element_to_rank_map[i]];
                }
            }

            // Finally, iterate through the sorted list of DoF combinations across ranks and update
            // the global numbering accordingly.  Create a vector of indices to mask dof_map.rank_to_global_map
            // that allows sorted access into dof_map.rank_to_global_map by the global number (disregards intervals).
            std::vector<std::size_t> indices_sorted_by_global(dof_map.rank_to_global_map.size());
            std::iota(indices_sorted_by_global.begin(), indices_sorted_by_global.end(), 0);
            const auto& rank_to_global_map_ref = dof_map.rank_to_global_map;
            std::sort(
                indices_sorted_by_global.begin(),
                indices_sorted_by_global.end(),
                [&rank_to_global_map_ref](std::size_t lhs_index, std::size_t rhs_index) -> bool
                {
                    return rank_to_global_map_ref[lhs_index] < rank_to_global_map_ref[rhs_index];
                });
            // Update the global numbers (accounting for DoF's eliminated from the inter-rank combinations)
            auto curr_global_dof_to_combine_iter = global_dofs_to_combine_map.begin();
            GlobalDofIndex curr_offset = 0;
            for (auto i : utility::range_from_zero(indices_sorted_by_global.size()))
            {
                auto& rank_index_ordered_by_global = indices_sorted_by_global[i];
                auto& curr_global_index = dof_map.rank_to_global_map[rank_index_ordered_by_global];

                if (curr_global_dof_to_combine_iter != global_dofs_to_combine_map.end() &&
                    curr_global_index >= curr_global_dof_to_combine_iter->first)
                {
                    assert(curr_global_index != curr_global_dof_to_combine_iter->first);
                    curr_offset++;
                    curr_global_dof_to_combine_iter++;
                }

                // Subtract the number of DoF's eliminated with lower global numbers
                curr_global_index -= curr_offset;
            }

            // Update the owned global range so sort_intervals can determine which DoF's are owned.
            dof_map.owned_global_dof_begin -= n_dofs_eliminated_on_lower_ranks;
            dof_map.owned_global_dof_end -= n_dofs_eliminated_on_lower_ranks + n_dofs_eliminated_on_this_rank;
        }

        std::vector<std::size_t> sort_intervals(DofMap& dof_map)
        {
            std::vector<DofType> dof_types_map(dof_map.rank_to_global_map.size(), DofType::EXCLUSIVE_GHOST);
            for (auto local_element_number : utility::range_from_zero(dof_map.element_to_rank_maps.size()))
            {
                auto& element_to_rank_map = dof_map.element_to_rank_maps[local_element_number];
                if (local_element_number < dof_map.n_owned_elements)
                {
                    for (auto i : utility::range_from_zero(dof_map.element_n_dofs[local_element_number]))
                    {
                        auto& rank_index = element_to_rank_map[i];
                        auto& global_index = dof_map.rank_to_global_map[rank_index];
                        if (global_index >= dof_map.owned_global_dof_begin &&
                            global_index <= dof_map.owned_global_dof_end)
                        {
                            dof_types_map[rank_index] = DofType::OWNED;
                        }
                        else
                        {
                            dof_types_map[rank_index] = DofType::LOCAL_GHOST;
                        }
                    }
                }
            }

            // Update counts
            dof_map.n_owned_dofs = std::count_if(
                dof_types_map.begin(),
                dof_types_map.end(),
                [](const DofType& type) -> bool
                {
                    return type == DofType::OWNED;
                });
            dof_map.n_local_ghost_dofs = std::count_if(
                dof_types_map.begin(),
                dof_types_map.end(),
                [](const DofType& type) -> bool
                {
                    return type == DofType::LOCAL_GHOST;
                });
            dof_map.n_exclusive_ghost_dofs = dof_types_map.size() -
                dof_map.n_owned_dofs - dof_map.n_local_ghost_dofs;


            // Sort the rank indices first by the DoF type then by the global index
            const auto& rank_to_global_map_ref = dof_map.rank_to_global_map;
            std::vector<std::size_t> backward_permutation_indices(dof_map.rank_to_global_map.size());
            std::iota(backward_permutation_indices.begin(), backward_permutation_indices.end(), 0);
            std::sort(
                backward_permutation_indices.begin(),
                backward_permutation_indices.end(),
                [&dof_types_map, &rank_to_global_map_ref](const std::size_t& lhs_index, const std::size_t& rhs_index) -> bool
                {
                    return std::tie(dof_types_map[lhs_index], rank_to_global_map_ref[lhs_index]) <
                           std::tie(dof_types_map[rhs_index], rank_to_global_map_ref[rhs_index]);
                });
            
            std::vector<std::size_t> forward_permutation_indices(dof_map.rank_to_global_map.size());
            for (auto i : utility::indices(backward_permutation_indices))
            {
                forward_permutation_indices[backward_permutation_indices[i]] = i;
            }

            // Apply permutation
            apply_permutation(
                dof_map.rank_to_global_map,
                backward_permutation_indices);
            for (auto local_element_number : utility::range_from_zero(dof_map.element_to_rank_maps.size()))
            {
                auto& element_to_rank_map = dof_map.element_to_rank_maps[local_element_number];
                for (auto i : utility::range_from_zero(dof_map.element_n_dofs[local_element_number]))
                {
                    element_to_rank_map[i] =
                        forward_permutation_indices[element_to_rank_map[i]];
                }
            }

            return backward_permutation_indices;
        }
    }
}