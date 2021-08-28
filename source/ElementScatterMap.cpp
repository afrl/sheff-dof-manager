#include "dof_manager/ElementScatterMap.h"

#include "utility/MPIUtils.h"

namespace dof_manager
{
namespace detail
{
void build_receive_maps(const std::set<std::pair<int, ElementLocalIndex>>& elements_to_receive_set,
                        ElementScatterMap& scatter_map)
{
    scatter_map.elements_to_receive.clear();
    scatter_map.rank_receive_offsets.clear();
    scatter_map.source_ranks.clear();

    if (elements_to_receive_set.empty())
    {
        return;
    }

    size_t curr_offset = 0;
    int curr_rank      = elements_to_receive_set.begin()->first;

    scatter_map.rank_receive_offsets.push_back(curr_offset);
    scatter_map.source_ranks.push_back(curr_rank);

    scatter_map.elements_to_receive.reserve(elements_to_receive_set.size());
    for (auto& rank_element_pair : elements_to_receive_set)
    {
        if (rank_element_pair.first != curr_rank)
        {
            curr_rank = rank_element_pair.first;
            scatter_map.rank_receive_offsets.push_back(curr_offset);
            scatter_map.source_ranks.push_back(curr_rank);
        }

        scatter_map.elements_to_receive.push_back(rank_element_pair.second);
        curr_offset++;
    }

    scatter_map.rank_receive_offsets.push_back(curr_offset);
}

void build_send_maps(const std::set<std::pair<int, ElementLocalIndex>>& elements_to_send_set,
                     ElementScatterMap& scatter_map)
{
    scatter_map.elements_to_send.clear();
    scatter_map.rank_send_offsets.clear();
    scatter_map.destination_ranks.clear();

    if (elements_to_send_set.empty())
    {
        return;
    }

    size_t curr_offset = 0;
    int curr_rank      = elements_to_send_set.begin()->first;

    scatter_map.rank_send_offsets.push_back(curr_offset);
    scatter_map.destination_ranks.push_back(curr_rank);

    scatter_map.elements_to_send.reserve(elements_to_send_set.size());
    for (auto& rank_element_pair : elements_to_send_set)
    {
        if (rank_element_pair.first != curr_rank)
        {
            curr_rank = rank_element_pair.first;
            scatter_map.rank_send_offsets.push_back(curr_offset);
            scatter_map.destination_ranks.push_back(curr_rank);
        }

        scatter_map.elements_to_send.push_back(rank_element_pair.second);
        curr_offset++;
    }

    scatter_map.rank_send_offsets.push_back(curr_offset);
}
}


detail::ScatterRankRange ElementScatterMap::destination_ranks_range() const
{
    return { destination_ranks };
}

detail::ScatterElementRange ElementScatterMap::elements_to_send_range(
  const detail::RankIndexValuePair& rank_pair) const
{
    return { elements_to_send.data() + rank_send_offsets[rank_pair.index],
             elements_to_send.data() + rank_send_offsets[rank_pair.index + 1] };
}

detail::ScatterRankRange ElementScatterMap::source_ranks_range() const
{
    return { source_ranks };
}

detail::ScatterElementRange ElementScatterMap::elements_to_receive_range(
  const detail::RankIndexValuePair& rank_pair) const
{
    return { elements_to_receive.data() + rank_receive_offsets[rank_pair.index],
             elements_to_receive.data() + rank_receive_offsets[rank_pair.index + 1] };
}

bool ElementScatterMap::initialized() const
{
    return destination_ranks.size() > 0 || !utility::MPIUtils::is_distributed();
}

void update_element_scatter_map(const finite_element::FiniteElementMesh& fe_mesh,
                                const mesh::ElementAdjacencyList& adjacency,
                                ElementScatterMap& scatter_map)
{
    const auto this_rank = utility::MPIUtils::get_local_rank();

    std::set<std::pair<int, ElementLocalIndex>> elements_to_receive_set;
    for (auto fe : fe_mesh.finite_elements())
    {
        // Receive ghost elements from the respective rank
        if (fe.element.parent_partition() != this_rank)
        {
            elements_to_receive_set.insert(
              { fe.element.parent_partition(), fe.element.local_index() });
        }

        // Receive any element (ghost or owned) that contains a ghost node from the respective
        // rank of the ghost node
        for (auto n : fe.element.nodes())
        {
            if (n.parent_partition() != this_rank)
            {
                elements_to_receive_set.insert({ n.parent_partition(), fe.element.local_index() });
            }
        }
    }

    detail::build_receive_maps(elements_to_receive_set, scatter_map);

    std::set<std::pair<int, ElementLocalIndex>> elements_to_send_set;
    for (auto fe : fe_mesh.finite_elements())
    {
        if (fe.element.parent_partition() == this_rank)
        {
            // Send any owned elements that are adjacent to a ghost element to the respective rank
            // of the ghost element.
            for (auto& neighbor : adjacency.get_adjacent_elements(fe.element.local_index()))
            {
                if (neighbor.parent_partition() != this_rank)
                {
                    elements_to_send_set.insert(
                      { neighbor.parent_partition(), fe.element.local_index() });
                }
            }
        }
        else
        {
            // Send any ghost element that contains an owned node to all ranks that own a node in
            // the element (all owned elements that could appear as a ghost on another rank are
            // already included).
            for (auto n : fe.element.nodes())
            {
                if (n.parent_partition() != this_rank)
                {
                    elements_to_send_set.insert({ n.parent_partition(), fe.element.local_index() });
                }
            }
        }
    }

    detail::build_send_maps(elements_to_send_set, scatter_map);
}

}
