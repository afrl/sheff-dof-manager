#include "dof_manager/DofMap.h"

#include "utility/MPIUtils.h"
#include "utility/Range.h"

#include "finite_element/BaseFormulation.h"

namespace dof_manager
{
    DofMap::DofMap(const MeshType & mesh)
        : mesh_(mesh),
        n_global_dofs_(0),
        n_all_dofs_on_rank_(0),
        phase_(DofMapPhase::STATIC)
    {
        enumerate_dofs();
    }

    DofMap::~DofMap()
    {
        for (auto& map : element_to_rank_maps_)
        {
            delete[] map;
        }
        for (auto& map : node_to_rank_maps_)
        {
            delete[] map;
        }
    }

    const std::vector<RankDofIndex*>& DofMap::get_element_to_rank_maps() const
    {
        return element_to_rank_maps_;
    }

    const std::vector<ElementDofIndex>& DofMap::get_element_n_dofs() const
    {
        return element_n_dofs_;
    }

    ElementToRankDofMap DofMap::get_element_map(
        const flex_mesh::LocalIndex element_local_number) const
    {
        return ElementToRankDofMap(
            element_to_rank_maps_[element_local_number],
            element_n_dofs_[element_local_number]);
    }

    const std::vector<RankDofIndex*>& DofMap::get_node_to_rank_maps() const
    {
        return node_to_rank_maps_;
    }

    const std::vector<ElementDofIndex>& DofMap::get_node_n_dofs() const
    {
        return node_n_dofs_;
    }

    NodeToRankDofMap DofMap::get_node_map(
        const flex_mesh::LocalIndex node_local_number) const
    {
        return NodeToRankDofMap(
            node_to_rank_maps_[node_local_number],
            node_n_dofs_[node_local_number]);
    }

    void DofMap::start_enrichment_phase()
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::STATIC,
            "This can only be called during a static phase.");

        phase_ = DofMapPhase::ENRICHMENT;
        ep_new_dof_to_old_rank_map_.clear();
    }

    const RankDofIndex DofMap::add_new_dof(
        const RankDofIndex original_sibling_dof)
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::ENRICHMENT,
            "This can only be called during an enrichment phase.");

        // Make sure if the sibling dof is also a new dof that we map back
        // to the old rank dof for the sibling
        if (original_sibling_dof >= n_all_dofs_on_rank_)
        {
            const auto old_rank_dof =
                ep_new_dof_to_old_rank_map_[original_sibling_dof - n_all_dofs_on_rank_];
            ep_new_dof_to_old_rank_map_.push_back(old_rank_dof);
        }
        else
        {
            ep_new_dof_to_old_rank_map_.push_back(original_sibling_dof);
        }

        const RankDofIndex new_rank_number =
            n_all_dofs_on_rank_ + static_cast<RankDofIndex>(ep_new_dof_to_old_rank_map_.size()) - 1;
        return new_rank_number;
    }

    void DofMap::enrich_element(
        const flex_mesh::LocalIndex& local_element_number,
        const std::vector<RankDofIndex>& replacement_dofs)
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::ENRICHMENT,
            "This can only be called during an enrichment phase.");

        auto curr_n_dofs_on_rank =
            n_all_dofs_on_rank_ + ep_new_dof_to_old_rank_map_.size();
        for (auto& replacement_rank_dof : replacement_dofs)
        {
            _UTIL_ASSERT_C(replacement_rank_dof < curr_n_dofs_on_rank,
                "One of the replacement DoF's was out of range.  Was add_new_dof() called first?");
        }

        delete element_to_rank_maps_[local_element_number];
        RankDofIndex *new_dofs = new RankDofIndex[replacement_dofs.size()];
        element_to_rank_maps_[local_element_number] = new_dofs;
        std::copy(replacement_dofs.begin(), replacement_dofs.end(), new_dofs);

        element_n_dofs_[local_element_number] =
            static_cast<RankDofIndex>(replacement_dofs.size());
    }

    void DofMap::enrich_node(
        const flex_mesh::LocalIndex& local_node_number,
        RankDofIndex new_rank_dof_start,
        ElementDofIndex n_added_dofs)
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::ENRICHMENT,
            "This can only be called during an enrichment phase.");

        auto curr_n_dofs_on_rank =
            n_all_dofs_on_rank_ + ep_new_dof_to_old_rank_map_.size();
        _UTIL_ASSERT_C(new_rank_dof_start + n_added_dofs <= curr_n_dofs_on_rank,
            "One of the new DoF's was out of range.  Was add_new_dof() called first?");

        const auto n_old_dofs_for_node = node_n_dofs_[local_node_number];
        const auto n_new_dofs_for_node = n_old_dofs_for_node + n_added_dofs;
        auto& node_dof_map = node_to_rank_maps_[local_node_number];

        // Resize the dof map for the node (keeping the older numbering for
        // i = [0, n_old_dofs_for_node)
        RankDofIndex* new_node_dof_map = new RankDofIndex[n_new_dofs_for_node];
        std::copy(node_dof_map, node_dof_map + n_old_dofs_for_node, new_node_dof_map);
        delete[] node_dof_map;
        node_dof_map = new_node_dof_map;
        node_n_dofs_[local_node_number] = n_new_dofs_for_node;

        // Set the new dof numbers
        for (auto i : utility::range_from_zero(n_added_dofs))
        {
            node_dof_map[i + n_old_dofs_for_node] = new_rank_dof_start + i;
        }
    }

    const std::vector<RankDofIndex>& DofMap::get_enrichment_new_dof_to_old_rank_map()
    {
        return ep_new_dof_to_old_rank_map_;
    }

    void DofMap::start_renumber_phase()
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::ENRICHMENT,
            "This can only be called during an enrichment phase.");

        renumber_dofs();
        phase_ = DofMapPhase::RENUMBER;
    }

    const std::vector<GlobalDofIndex>& DofMap::get_renumbering_old_rank_to_global_map() const
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::RENUMBER,
            "This can only be called during a renumbering phase.");

        return rp_old_rank_to_global_map_;
    }

    const std::vector<RankDofIndex>& DofMap::get_renumbering_new_rank_to_old_rank_map() const
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::RENUMBER,
            "This can only be called during a renumbering phase.");

        return rp_new_rank_to_old_rank_map_;
    }

    void DofMap::end_renumber_phase()
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::RENUMBER,
            "This can only be called during a renumbering phase.");

        rp_new_rank_to_old_rank_map_.clear();
        rp_old_rank_to_global_map_.clear();
        phase_ = DofMapPhase::STATIC;
    }

    const std::string DofMap::str() const
    {
        std::stringstream ss;
        ss << "Element DoF Map: (" << element_n_dofs_.size() << " elements)" << std::endl;
        for (auto i : utility::range_from_zero(element_n_dofs_.size()))
        {
            for (auto j : utility::range_from_zero(element_n_dofs_[i]))
            {
                ss << " " << element_to_rank_maps_[i][j];
            }
            ss << std::endl;
        }

        ss << "Node DoF Map: (" << node_n_dofs_.size() << " nodes)" << std::endl;
        for (auto i : utility::range_from_zero(node_n_dofs_.size()))
        {
            for (auto j : utility::range_from_zero(node_n_dofs_[i]))
            {
                ss << " " << node_to_rank_maps_[i][j];
            }
            ss << std::endl;
        }


        return ss.str();
    }


    void DofMap::enumerate_dofs()
    {
        // For now, assume each node has the same number of DoF's at the start of the analysis.
        // This algorithm will have to be generalized if this is not the case.
        auto initial_dofs_per_node = mesh_.elements().front()->get_formulation_const().get_n_dofs_per_node();

        element_to_rank_maps_.resize(mesh_.elements().size(), nullptr);
        element_n_dofs_.resize(mesh_.elements().size(), 0);

        node_to_rank_maps_.resize(mesh_.nodes().size(), nullptr);
        node_n_dofs_.resize(mesh_.nodes().size(), initial_dofs_per_node);

        // Count the number of DoF's on this rank in each interval
        n_owned_dofs_ = 0;
        n_lower_ghost_dofs_ = 0;
        n_higher_ghost_dofs_ = 0;
        for (auto& n_ptr : mesh_.nodes())
        {
            if (n_ptr->get_parent_partition() < utility::MPIUtils::get_local_rank())
            {
                n_lower_ghost_dofs_ += initial_dofs_per_node;
            }
            else if (n_ptr->get_parent_partition() == utility::MPIUtils::get_local_rank())
            {
                n_owned_dofs_ += initial_dofs_per_node;
            }
            else
            {
                n_higher_ghost_dofs_ += initial_dofs_per_node;
            }
        }
        n_all_dofs_on_rank_ = n_lower_ghost_dofs_ + n_owned_dofs_ + n_all_dofs_on_rank_;

        // Create the initial node-to-rank maps and rank-to-global map
        // When creating the rank to global map, it is assumed that the node
        // global numbering follows:
        // 1) All owned node numbers by a rank is a continuous interval of integers
        // 2) The intervals owned by each rank are sorted by rank
        rank_to_global_map_.resize(n_all_dofs_on_rank_, 0);
        RankDofIndex next_pre_ghost_index = 0;
        RankDofIndex next_owned_index = n_lower_ghost_dofs_;
        RankDofIndex next_post_ghost_index = n_lower_ghost_dofs_ + n_owned_dofs_;
        for (auto& n_ptr : mesh_.nodes())
        {
            auto& node_dof_map = node_to_rank_maps_[n_ptr->get_local_number()];
            node_dof_map = new RankDofIndex[initial_dofs_per_node];
            if (n_ptr->get_parent_partition() < utility::MPIUtils::get_local_rank())
            {
                for (auto component : utility::range_from_zero(initial_dofs_per_node))
                {
                    const auto global_number =
                        n_ptr->get_global_number() * initial_dofs_per_node + component;
                    rank_to_global_map_[next_pre_ghost_index] = global_number;
                    node_dof_map[component] = next_pre_ghost_index;
                    next_pre_ghost_index++;
                }
            }
            else if (n_ptr->get_parent_partition() == utility::MPIUtils::get_local_rank())
            {
                for (auto component : utility::range_from_zero(initial_dofs_per_node))
                {
                    const auto global_number =
                        n_ptr->get_global_number() * initial_dofs_per_node + component;
                    rank_to_global_map_[next_owned_index] = global_number;
                    node_dof_map[component] = next_owned_index;
                    next_owned_index++;
                }
            }
            else
            {
                for (auto component : utility::range_from_zero(initial_dofs_per_node))
                {
                    const auto global_number =
                        n_ptr->get_global_number() * initial_dofs_per_node + component;
                    rank_to_global_map_[next_post_ghost_index] = global_number;
                    node_dof_map[component] = next_post_ghost_index;
                    next_post_ghost_index++;
                }
            }
        }

        // Create the initial element-rank-map
        n_owned_elements_ = 0;
        const auto this_rank = utility::MPIUtils::get_local_rank();
        for (auto& e_ptr : mesh_.elements())
        {
            const ElementDofIndex n_dofs_for_element =
                static_cast<ElementDofIndex>(e_ptr->get_num_nodes() * initial_dofs_per_node);
            element_to_rank_maps_[e_ptr->get_local_number()] =
                new RankDofIndex[n_dofs_for_element];
            element_n_dofs_[e_ptr->get_local_number()] = n_dofs_for_element;

            auto& element_dof_map = element_to_rank_maps_[e_ptr->get_local_number()];
            ElementDofIndex node_index = 0;
            for (auto& n_ptr : *e_ptr)
            {
                for (auto component : utility::range_from_zero(initial_dofs_per_node))
                {
                    const auto rank_number =
                        node_to_rank_maps_[n_ptr->get_local_number()][component];
                    element_dof_map[node_index * initial_dofs_per_node + component] =
                        rank_number;
                }
                node_index++;
            }

            if (e_ptr->get_parent_partition() == this_rank)
            {
                n_owned_elements_++;
            }
        }

        auto n_owned_dofs_by_rank = utility::MPIUtils::all_gather_vectors<GlobalDofIndex>(
            { static_cast<GlobalDofIndex>(n_owned_dofs_) });

        // Determine the range of dof's this rank owns
        // We assume that each rank will own a contiguous range, so we can determined
        // the owned range of this rank based on the n_owned_dofs in each rank.
        owned_global_dof_begin_ = 0;
        for (auto rank : utility::range_from_zero(utility::MPIUtils::get_local_rank()))
        {
            owned_global_dof_begin_ += n_owned_dofs_by_rank[rank];
        }
        owned_global_dof_end_ = owned_global_dof_begin_ + n_owned_dofs_;

        n_global_dofs_ = 0;
        for (auto n_owned_dofs_for_rank : n_owned_dofs_by_rank)
        {
            n_global_dofs_ += n_owned_dofs_for_rank;
        }
    }

    void DofMap::renumber_dofs()
    {
        // Cache some old counts
        const auto old_n_all_dofs_on_rank = n_all_dofs_on_rank_;
        const auto old_n_owned_and_pre_ghost_dofs =
            n_lower_ghost_dofs_ + n_owned_dofs_;
        const auto old_n_owned_dofs = n_owned_dofs_;

        // For now assume there are no new ghosts, but this will need to be
        // updated if cracks are allowed to grow up to partition boundaries.
        // To alleviate this restriction, scatter any new DoF's to ranks that
        // have the original sibling DoF as a ghost.

        // Find how many new DoF's were created on each rank
        const auto n_new_dofs_for_this_rank = static_cast<RankDofIndex>(
            ep_new_dof_to_old_rank_map_.size());
        auto new_dofs_by_rank =
            utility::MPIUtils::all_gather_vectors<RankDofIndex>({ n_new_dofs_for_this_rank });
        auto n_global_new_dofs = std::accumulate(new_dofs_by_rank.begin(), new_dofs_by_rank.end(), 0);
        auto n_new_dofs_from_lower_ranks = std::accumulate(
            new_dofs_by_rank.begin(),
            new_dofs_by_rank.begin() + utility::MPIUtils::get_local_rank(),
            0);
        std::vector<RankDofIndex> n_new_dofs_from_lower_ranks_by_rank;
        {
            RankDofIndex cumulative_new_dof_count = 0;
            for (auto n_new_dofs_for_rank : new_dofs_by_rank)
            {
                n_new_dofs_from_lower_ranks_by_rank.push_back(cumulative_new_dof_count);
                cumulative_new_dof_count += n_new_dofs_for_rank;
            }
        }

        // Find out how many DoF's each rank used to own and the beginning of each range
        auto old_n_owned_dofs_by_rank =
            utility::MPIUtils::all_gather_vectors<RankDofIndex>({ n_owned_dofs_ });
        std::vector<GlobalDofIndex> old_owned_global_dof_begin_by_rank;
        {
            GlobalDofIndex cumulative_dof_count = 0;
            for (auto n_owned_dofs_for_rank : old_n_owned_dofs_by_rank)
            {
                old_owned_global_dof_begin_by_rank.push_back(cumulative_dof_count);
                cumulative_dof_count += n_owned_dofs_for_rank;
            }
        }

        // Update all of the counts
        n_global_dofs_ += n_global_new_dofs;
        n_all_dofs_on_rank_ += n_new_dofs_for_this_rank;
        n_owned_dofs_ += n_new_dofs_for_this_rank;
        owned_global_dof_begin_ += n_new_dofs_from_lower_ranks;
        owned_global_dof_end_ += n_new_dofs_from_lower_ranks + n_new_dofs_for_this_rank;
        // n_higher_ghost_dofs_ is unchanged since we assume no new ghosts
        // n_lower_ghost_dofs_ is unchanged since we assume no new ghosts

        // Create the new rank to old rank map
        rp_new_rank_to_old_rank_map_.resize(n_all_dofs_on_rank_);
        for (auto new_rank_index : utility::range_from_zero(
            n_lower_ghost_dofs_ + old_n_owned_dofs))
        {
            // WARNING: Assumes n_lower_ghost_dofs_ is unchanged
            rp_new_rank_to_old_rank_map_[new_rank_index] = new_rank_index;
        }
        // Insert the new DoF's at the end of the owned range
        for (auto new_dof_index : utility::range_from_zero(n_new_dofs_for_this_rank))
        {
            // WARNING: Assumes n_lower_ghost_dofs_ is unchanged
            auto const new_rank_index =
                n_lower_ghost_dofs_ + old_n_owned_dofs + new_dof_index;
            rp_new_rank_to_old_rank_map_[new_rank_index] =
                ep_new_dof_to_old_rank_map_[new_dof_index];
        }
        for (auto new_rank_index : utility::range(
            n_lower_ghost_dofs_ + n_owned_dofs_ + n_new_dofs_for_this_rank,
            n_lower_ghost_dofs_ + n_owned_dofs_ + n_new_dofs_for_this_rank + n_higher_ghost_dofs_))
        {
            // WARNING: Assumes n_lower_ghost_dofs_ and n_higher_ghost_dofs_ is unchanged
            rp_new_rank_to_old_rank_map_[new_rank_index] = new_rank_index - n_new_dofs_for_this_rank;
        }
        ep_new_dof_to_old_rank_map_.clear();

        // Create the new rank-to-global map
        rp_old_rank_to_global_map_.swap(rank_to_global_map_);
        rank_to_global_map_.resize(n_all_dofs_on_rank_);
        // Pre-ghost interval
        // WARNING: Assumes n_lower_ghost_dofs_ is unchanged
        for (auto i : utility::range_from_zero(n_lower_ghost_dofs_))
        {
            const auto old_global_index = rp_old_rank_to_global_map_[i];
            // Find which rank the old global index belonged to
            for (auto rank : utility::range_from_zero(utility::MPIUtils::get_local_rank()))
            {
                // Due to the sequential ordering, the first rank that index is
                // greater than is the rank that owns it.
                if (old_global_index > old_owned_global_dof_begin_by_rank[rank])
                {
                    rank_to_global_map_[i] =
                        old_global_index + n_new_dofs_from_lower_ranks_by_rank[rank];
                }
            }
        }
        // Owned interval before enrichment
        for (auto i : utility::range(n_lower_ghost_dofs_,
            n_lower_ghost_dofs_ + old_n_owned_dofs))
        {
            const auto old_rank_index = rp_new_rank_to_old_rank_map_[i];
            const auto old_global_index = rp_old_rank_to_global_map_[old_rank_index];
            rank_to_global_map_[i] =
                old_global_index + n_new_dofs_from_lower_ranks;
        }
		// New owned DoF's interval
		for (auto i : utility::range_from_zero(n_new_dofs_for_this_rank))
		{
			rank_to_global_map_[i + n_lower_ghost_dofs_ + old_n_owned_dofs] =
				i + rp_old_rank_to_global_map_.back() + n_new_dofs_from_lower_ranks + 1;
		}
        // Post-ghost interval
        // WARNING: Assumes n_lower_ghost_dofs_ and n_higher_ghost_dofs_ is unchanged
        for (auto i : utility::range(n_lower_ghost_dofs_ + n_owned_dofs_,
            n_lower_ghost_dofs_ + n_owned_dofs_ + n_higher_ghost_dofs_))
        {
            const auto old_rank_index = rp_new_rank_to_old_rank_map_[i];
            const auto old_global_index = rp_old_rank_to_global_map_[old_rank_index];
            // Find which rank the old global index belonged to
            for (auto rank : utility::range(
                utility::MPIUtils::get_local_rank() + 1,
                utility::MPIUtils::get_num_local_ranks()))
            {
                // Due to the sequential ordering, the first rank that index is
                // greater than is the rank that owns it.
                if (old_global_index > old_owned_global_dof_begin_by_rank[rank])
                {
                    rank_to_global_map_[i] =
                        old_global_index + n_new_dofs_from_lower_ranks_by_rank[rank];
                }
            }
        }

        // Finally, update the element-to-rank and node-to-rank maps, since the rank numbering
        // during enrichment was:
        //  [ pre-ghosts  ] ‾‾\
        //  [    owned    ]    | is of length: old_n_all_dofs_on_rank
        //  [ post-ghosts ] __/ 
        //  [  new owned  ]
        // but we need to change it to be:
        //  [ pre-ghosts  ] ‾‾\
        //  [    owned    ] __/ is of length: old_n_owned_and_pre_ghost_dofs
        //  [  new owned  ]
        //  [ post-ghosts ]
        // We could generalize the renumbering to take a full rank-to-rank map, but
        // for this particular permutation, it is cheaper to just use offsets.
        for (auto local_element_number : utility::range_from_zero(element_to_rank_maps_.size()))
        {
            auto& element_to_rank_map = element_to_rank_maps_[local_element_number];
            for (auto i : utility::range_from_zero(element_n_dofs_[local_element_number]))
            {
                if (element_to_rank_map[i] > old_n_owned_and_pre_ghost_dofs)
                {
                    // A new dof index, which needs to be moved to the end of the
                    // owned interval (before post-ghosts).
                    if (element_to_rank_map[i] > old_n_all_dofs_on_rank)
                    {
                        element_to_rank_map[i] -= n_higher_ghost_dofs_;
                    }
					// A post-ghost index, which needs to be moved to the end (due to
					// new dof's on this rank)
					else if (element_to_rank_map[i] > old_n_owned_and_pre_ghost_dofs)
					{
						element_to_rank_map[i] += n_new_dofs_for_this_rank;
					}
                }
            }
        }
        for (auto local_node_number : utility::range_from_zero(node_to_rank_maps_.size()))
        {
            auto& node_to_rank_map = node_to_rank_maps_[local_node_number];
            for (auto i : utility::range_from_zero(node_n_dofs_[local_node_number]))
            {
                if (node_to_rank_map[i] > old_n_owned_and_pre_ghost_dofs)
                {
					// A new dof index, which needs to be moved to the end of the
					// owned interval (before post-ghosts).
					if (node_to_rank_map[i] > old_n_all_dofs_on_rank)
					{
						node_to_rank_map[i] -= n_higher_ghost_dofs_;
					}
					// A post-ghost index, which needs to be moved to the end (due to
					// new dof's on this rank)
					else if (node_to_rank_map[i] > old_n_owned_and_pre_ghost_dofs)
					{
						node_to_rank_map[i] += n_new_dofs_for_this_rank;
					}
                }
            }
        }
    }

}
