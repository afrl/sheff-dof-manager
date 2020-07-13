#include "dof_manager/NodeScatterMap.h"

#include "dof_manager/ScatterMapDestBuilder.h"
#include "utility/MPIUtils.h"

namespace dof_manager
{
    NodeScatterMap::NodeScatterMap(
        const MeshType& mesh,
        const ElementScatterMap& element_scatter_map)
    {
        build_maps(mesh, element_scatter_map);
    }

    detail::ScatterRankRange dof_manager::NodeScatterMap::destination_ranks_range() const
    {
        return { destination_ranks };
    }

    detail::ScatterNodeRange dof_manager::NodeScatterMap::nodes_to_send_range(
        const detail::RankIndexValuePair& rank_pair) const
    {
        return {
            nodes_to_send.data() + rank_send_offsets[rank_pair.index],
            nodes_to_send.data() + rank_send_offsets[rank_pair.index + 1]
        };
    }

    detail::ScatterRankRange NodeScatterMap::source_ranks_range() const
    {
        return { source_ranks };
    }
    detail::ScatterNodeRange NodeScatterMap::nodes_to_receive_range(
        const detail::RankIndexValuePair& rank_pair) const
    {
        return {
            nodes_to_receive.data() + rank_receive_offsets[rank_pair.index],
            nodes_to_receive.data() + rank_receive_offsets[rank_pair.index + 1]
        };
    }

    void NodeScatterMap::build_maps(
        const MeshType& mesh,
        const ElementScatterMap& element_scatter_map)
    {
        const auto this_rank = utility::MPIUtils::get_local_rank();

        // The node scatter map does not neccessary have the same source/dest ranks for exchanging
        // information.  It is possible that a rank has a node from a specific rank but not an
        // element. So build the correct maps through MPI communication.
        ScatterMapDestBuilder dest_builder;
        for (auto& n_ptr : mesh.nodes())
        {
            if (n_ptr->get_parent_partition() != this_rank)
            {
                dest_builder.add_source_rank(n_ptr->get_parent_partition());
            }
        }
        destination_ranks = dest_builder.get_dest_ranks();
        source_ranks = dest_builder.get_source_ranks();

        // Step 1: Communicate the ghost nodes this rank has to the respective remote rank,
        //         and build the receive maps.
        std::vector<NodeLocalIndex> send_buffer;
        std::vector<size_t> send_offsets = { 0 };
        rank_receive_offsets = { 0 };
        for (auto dest : element_scatter_map.destination_ranks_range())
        {
            auto dest_rank = dest.rank;

            for (auto& n_ptr : mesh.nodes())
            {
                if (n_ptr->get_parent_partition() == dest_rank)
                {
                    send_buffer.push_back(n_ptr->get_global_number());
                    nodes_to_receive.push_back(n_ptr->get_local_number());
                }
            }
            send_offsets.push_back(send_buffer.size());
            rank_receive_offsets.push_back(nodes_to_receive.size());
        }
        send_offsets.push_back(send_buffer.size());
        rank_receive_offsets.push_back(nodes_to_receive.size());

        // Step 2: Exchange the lists of ghost nodes with ghost layers
        std::vector<GlobalDofIndex> exchange_buffer;
        utility::MPIUtils::exchange_arrays<GlobalDofIndex, size_t>(
            static_cast<int>(destination_ranks.size()),
            destination_ranks.data(),
            send_buffer.data(),
            send_offsets.data(),
            static_cast<int>(source_ranks.size()),
            source_ranks.data(),
            exchange_buffer,
            rank_send_offsets);

        // Step 3: Build a ghost-node global -> local number map to remap the receive buffer
        //         to the local rank numbering
        std::map<flex_mesh::GlobalIndex, flex_mesh::LocalIndex> owned_node_global_to_local_id_map;
        for (auto& n_ptr : mesh.nodes())
        {
            if (n_ptr->get_parent_partition() == this_rank)
            {
                owned_node_global_to_local_id_map[n_ptr->get_global_number()] = n_ptr->get_local_number();
            }
        }

        // Step 4: Populate the send maps
        for (auto source : element_scatter_map.source_ranks_range())
        {
            for (auto offset = rank_send_offsets[source.index];
                 offset < rank_send_offsets[source.index + 1]; )
            {
                const auto& node_global_id = exchange_buffer[offset++];

                auto find_iter = owned_node_global_to_local_id_map.find(node_global_id);
                assert(find_iter != owned_node_global_to_local_id_map.end());
                const auto& node_local_id = find_iter->second;

                nodes_to_send.push_back(node_local_id);
            }
        }
    }
}