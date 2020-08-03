#include "dof_manager/NodeScatterMap.h"

#include "dof_manager/ScatterMapDestBuilder.h"
#include "utility/MPIUtils.h"

#include <fstream>

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

        //std::ofstream out("build_maps_" + std::to_string(this_rank) + ".txt");

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
        /*
        out << this_rank << ": Finished building dest ranks: ";
        for (auto& r : destination_ranks) out << r << " ";
        out << " with sources: ";
        for (auto& r : source_ranks) out << r << " ";
        out << std::endl;
        */

        // Step 1: Communicate the ghost nodes this rank has to the respective remote rank,
        //         and build the receive maps.
        std::vector<flex_mesh::GlobalIndex> send_buffer;
        std::vector<size_t> send_offsets = { 0 };
        rank_receive_offsets = { 0 };
        for (auto source_rank : source_ranks)
        {
            //out << "Requests following nodes from " << source_rank << ":\n";
            for (auto& n_ptr : mesh.nodes())
            {
                if (n_ptr->get_parent_partition() == source_rank)
                {
                    send_buffer.push_back(n_ptr->get_global_number());
                    nodes_to_receive.push_back(n_ptr->get_local_number());
                    //out << n_ptr->get_global_number() << " ";
                }
            }
            send_offsets.push_back(send_buffer.size());
            rank_receive_offsets.push_back(nodes_to_receive.size());
            //out << std::endl;
        }
        send_offsets.push_back(send_buffer.size());
        rank_receive_offsets.push_back(nodes_to_receive.size());

        //out << this_rank << ": Starting exchange arrays" << std::endl;

        // Step 2: Exchange the lists of ghost nodes with ghost layers
        std::vector<flex_mesh::GlobalIndex> exchange_buffer;
        utility::MPIUtils::exchange_arrays<flex_mesh::GlobalIndex, size_t>(
            static_cast<int>(source_ranks.size()),
            source_ranks.data(),
            send_buffer.data(),
            send_offsets.data(),
            static_cast<int>(destination_ranks.size()),
            destination_ranks.data(),
            exchange_buffer,
            rank_send_offsets);
        assert(rank_send_offsets.size() == destination_ranks.size() + 1);
        /*
        out << this_rank << ": Finished exchange arrays" << std::endl;

        out << "exchange_offsets: { ";
        for (auto& v : rank_send_offsets) out << v << " ";
        out << "}" << std::endl;

        out << "exchange_data: { ";
        for (auto& v : exchange_buffer) out << v << " ";
        out << "}" << std::endl;
        */

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
        for (auto i : utility::indices(destination_ranks))
        {
            const auto dest_rank = destination_ranks[i];
            //out << "Owned nodes requested by " << dest_rank << ": (" << rank_send_offsets[i] << "-" << rank_send_offsets[i + 1] << ")\n";
            for (auto offset = rank_send_offsets[i];
                 offset < rank_send_offsets[i + 1]; )
            {
                const auto& node_global_id = exchange_buffer[offset++];
                //out << node_global_id << " ";

                auto find_iter = owned_node_global_to_local_id_map.find(node_global_id);
                if (find_iter == owned_node_global_to_local_id_map.end())
                {
                    //out << "\nError: " << node_global_id << " does not exist in the owned global to local map." << std::endl;
                    //out << offset << " " << rank_send_offsets[i] << " " << rank_send_offsets[i + 1] << " ";
                    //out.close();
                    assert(find_iter != owned_node_global_to_local_id_map.end());
                }
                const auto& node_local_id = find_iter->second;

                nodes_to_send.push_back(node_local_id);
            }
            //out << std::endl;
        }

        //out.close();
    }
}