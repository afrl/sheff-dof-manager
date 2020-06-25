#include "dof_manager/ElementScatterMap.h"

#include "utility/MPIUtils.h"

namespace dof_manager
{
    ElementScatterMap::ElementScatterMap(
        const MeshType& mesh, 
        const ElementAdjacencyList& adjacency)
    {
        build_receive_maps(mesh);
        build_send_maps(mesh, adjacency);
    }

    detail::ScatterRankRange ElementScatterMap::destination_ranks_range() const
    {
        return { destination_ranks };
    }

    detail::ScatterElementRange ElementScatterMap::elements_to_send_range(
        const detail::RankIndexValuePair& rank_pair) const
    {
        return {
            elements_to_send.data() + rank_send_offsets[rank_pair.index],
            elements_to_send.data() + rank_send_offsets[rank_pair.index + 1]
        };
    }

    detail::ScatterRankRange ElementScatterMap::source_ranks_range() const
    {
        return { source_ranks };
    }

    detail::ScatterElementRange ElementScatterMap::elements_to_receive_range(
        const detail::RankIndexValuePair& rank_pair) const
    {
        return {
            elements_to_receive.data() + rank_receive_offsets[rank_pair.index],
            elements_to_receive.data() + rank_receive_offsets[rank_pair.index + 1]
        };
    }

    void ElementScatterMap::build_receive_maps(const MeshType& mesh)
    {
        const auto this_rank = utility::MPIUtils::get_local_rank();

        // The set maintain a list of unique pairs sorted first by partition then by
        // local element number.
        std::set<std::pair<int, ElementLocalIndex>> elements_to_receive_set;
        for (auto& e_ptr : mesh.elements())
        {
            if (e_ptr->get_parent_partition() != this_rank)
            {
                elements_to_receive_set.insert({
                    e_ptr->get_parent_partition(),
                    e_ptr->get_local_number() });
            }
        }

        elements_to_receive.clear();
        rank_receive_offsets.clear();
        source_ranks.clear();

        if (elements_to_receive_set.empty())
        {
            return;
        }

        size_t curr_offset = 0;
        int curr_rank = elements_to_receive_set.begin()->first;

        rank_receive_offsets.push_back(curr_offset);
        source_ranks.push_back(curr_rank);

        elements_to_receive.reserve(elements_to_receive_set.size());
        for (auto& rank_element_pair : elements_to_receive_set)
        {
            if (rank_element_pair.first != curr_rank)
            {
                curr_rank = rank_element_pair.first;
                rank_receive_offsets.push_back(curr_offset);
                source_ranks.push_back(curr_rank);
            }

            elements_to_receive.push_back(rank_element_pair.second);
            curr_offset++;
        }

        rank_receive_offsets.push_back(curr_offset);
    }

    void ElementScatterMap::build_send_maps(
        const MeshType& mesh,
        const ElementAdjacencyList& adjacency)
    {
        const auto this_rank = utility::MPIUtils::get_local_rank();

        // The set maintain a list of unique pairs sorted first by partition then by
        // local element number.
        std::set<std::pair<int, ElementLocalIndex>> elements_to_send_set;
        for (auto& e_ptr : mesh.elements())
        {
            if (e_ptr->get_parent_partition() == this_rank)
            {
                for (auto& neighbor : adjacency.get_neighbors(e_ptr->get_local_number()))
                {
                    if (neighbor->get_parent_partition() != this_rank)
                    {
                        elements_to_send_set.insert({
                            neighbor->get_parent_partition(),
                            e_ptr->get_local_number() });
                    }
                }
            }
        }

        elements_to_send.clear();
        rank_send_offsets.clear();
        destination_ranks.clear();

        if (elements_to_send_set.empty())
        {
            return;
        }

        size_t curr_offset = 0;
        int curr_rank = elements_to_send_set.begin()->first;

        rank_send_offsets.push_back(curr_offset);
        destination_ranks.push_back(curr_rank);

        elements_to_send.reserve(elements_to_send_set.size());
        for (auto& rank_element_pair : elements_to_send_set)
        {
            if (rank_element_pair.first != curr_rank)
            {
                curr_rank = rank_element_pair.first;
                rank_send_offsets.push_back(curr_offset);
                destination_ranks.push_back(curr_rank);
            }

            elements_to_send.push_back(rank_element_pair.second);
            curr_offset++;
        }

        rank_send_offsets.push_back(curr_offset);
    }
}

