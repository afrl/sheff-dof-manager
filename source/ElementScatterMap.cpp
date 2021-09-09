#include "dof_manager/ElementScatterMap.h"

#include "mesh/algorithms/IO.h"
#include "utility/MPIUtils.h"

#include <fstream>
#include <iostream>

namespace dof_manager
{
namespace detail
{
void build_receive_maps(
  const std::set<std::tuple<int, mesh::GlobalIndex, ElementLocalIndex>>& elements_to_receive_set,
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
    int curr_rank      = std::get<0>(*elements_to_receive_set.begin());

    scatter_map.rank_receive_offsets.push_back(curr_offset);
    scatter_map.source_ranks.push_back(curr_rank);

    scatter_map.elements_to_receive.reserve(elements_to_receive_set.size());
    for (auto& rank_element_tuple : elements_to_receive_set)
    {
        if (std::get<0>(rank_element_tuple) != curr_rank)
        {
            curr_rank = std::get<0>(rank_element_tuple);
            scatter_map.rank_receive_offsets.push_back(curr_offset);
            scatter_map.source_ranks.push_back(curr_rank);
        }

        scatter_map.elements_to_receive.push_back(std::get<2>(rank_element_tuple));
        curr_offset++;
    }

    scatter_map.rank_receive_offsets.push_back(curr_offset);
}

void build_send_maps(
  const std::set<std::tuple<int, mesh::GlobalIndex, ElementLocalIndex>>& elements_to_send_set,
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
    int curr_rank      = std::get<0>(*elements_to_send_set.begin());

    scatter_map.rank_send_offsets.push_back(curr_offset);
    scatter_map.destination_ranks.push_back(curr_rank);

    scatter_map.elements_to_send.reserve(elements_to_send_set.size());
    for (auto& rank_element_tuple : elements_to_send_set)
    {
        if (std::get<0>(rank_element_tuple) != curr_rank)
        {
            curr_rank = std::get<0>(rank_element_tuple);
            scatter_map.rank_send_offsets.push_back(curr_offset);
            scatter_map.destination_ranks.push_back(curr_rank);
        }

        scatter_map.elements_to_send.push_back(std::get<2>(rank_element_tuple));
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

    std::set<std::tuple<int, mesh::GlobalIndex, ElementLocalIndex>> elements_to_receive_set;
    for (auto fe : fe_mesh.finite_elements())
    {
        // Receive ghost elements from the respective rank
        if (fe.element.parent_partition() != this_rank)
        {
            elements_to_receive_set.insert({ fe.element.parent_partition(),
                                             fe.element.global_index(),
                                             fe.element.local_index() });
        }

        // Receive any element (ghost or owned) that contains a ghost node from the respective
        // rank of the ghost node
        for (auto n : fe.element.nodes())
        {
            if (n.parent_partition() != this_rank)
            {
                elements_to_receive_set.insert(
                  { n.parent_partition(), fe.element.global_index(), fe.element.local_index() });
            }
        }
    }

    {
        if (this_rank == 1)
        {
            mesh::write_vtk(fe_mesh.mesh, "rank_1_part.vtk", false);
        }

        /*
        std::ofstream problem_out("e_receive_" + std::to_string(this_rank) + ".txt");
        problem_out << "Rank local_e_index global_e_index" << std::endl;
        for (auto& v : elements_to_receive_set)
        {
            problem_out << std::get<0>(v) << " " << std::get<2>(v) << " "
                        << fe_mesh.mesh.elements()[std::get<2>(v)].global_index() << std::endl;
        }
        problem_out.close();
        */
    }

    detail::build_receive_maps(elements_to_receive_set, scatter_map);

    // For relatively structured grids and some specific cases, it is possible to figure out what
    // elements need to be sent remote ranks (and which remote ranks require them).  However, in
    // general, I (Keith) have found many cases where it is impossible to figure it out.  The
    // simplest case I could come up with is:
    // 0      1      1      2
    // o------o------o------o
    // |      |      |      |   (Parent ranks labeled)
    // |   0  |   1  |   2  |
    // o------o------o------o
    // 0      0      1      2
    //
    // Rank 0 will not even have any elements from rank 2.  It will not know anything about rank 2,
    // yet Rank 2 will require rank 0 to send the middle element to it in order to enumerate the
    // DoFs correctly (since rank 0 owns one of the nodes in the middle element).
    //
    // However, each rank knows the remote ranks it needs elements from and which element from each
    // respective remote rank.  So the information can be communicated via MPI for each rank to find
    // the which elements to send to remote ranks.
    scatter_map.destination_ranks =
      utility::MPIUtils::find_exchange_destination_ranks(scatter_map.source_ranks);

    scatter_map.elements_to_send.clear();
    scatter_map.rank_send_offsets.clear();

    std::vector<mesh::LocalIndex> elements_to_receive_global_indices(
      scatter_map.elements_to_receive.size());
    for (auto i : utility::indices(scatter_map.elements_to_receive))
    {
        elements_to_receive_global_indices[i] =
          fe_mesh.mesh.elements()[scatter_map.elements_to_receive[i]].global_index();
    }

    // Exchange the global indices of the elements needed on this rank with ghost layers, in order
    // to figure out what elements to send to other ranks.
    std::vector<mesh::GlobalIndex> receive_buffer;
    utility::MPIUtils::exchange_arrays<mesh::GlobalIndex, size_t>(
      static_cast<int>(scatter_map.source_ranks.size()),
      scatter_map.source_ranks.data(),
      elements_to_receive_global_indices.data(),
      scatter_map.rank_receive_offsets.data(),
      static_cast<int>(scatter_map.destination_ranks.size()),
      scatter_map.destination_ranks.data(),
      receive_buffer,
      scatter_map.rank_send_offsets);

    // At this point, receive_buffer contains the global indices, map the global indices to the
    // local indices for the scatter map.
    std::unordered_map<mesh::GlobalIndex, mesh::LocalIndex> global_to_local_element_index_map;
    for (auto e : fe_mesh.mesh.elements())
    {
        global_to_local_element_index_map[e.global_index()] = e.local_index();
    }
    scatter_map.elements_to_send.resize(receive_buffer.size());
    for (auto i : utility::indices(scatter_map.elements_to_send))
    {
        scatter_map.elements_to_send[i] = global_to_local_element_index_map[receive_buffer[i]];
    }
}

void dump_scatter_map(const std::string& filepath,
                      const ElementScatterMap& scatter_map,
                      const mesh::Mesh& mesh)
{
    std::ofstream out(filepath);
    for (auto dest_rank : scatter_map.destination_ranks_range())
    {
        out << "Destination rank: " << dest_rank.rank << std::endl;
        for (auto local_element_index : scatter_map.elements_to_send_range(dest_rank))
        {
            out << "g " << mesh.elements()[local_element_index].global_index() << " l "
                << local_element_index << std::endl;
        }
    }
    for (auto source_rank : scatter_map.source_ranks_range())
    {
        out << "Source rank: " << source_rank.rank << std::endl;
        for (auto local_element_index : scatter_map.elements_to_receive_range(source_rank))
        {
            out << "g " << mesh.elements()[local_element_index].global_index() << " l "
                << local_element_index << std::endl;
        }
    }
    out.close();
}

}
