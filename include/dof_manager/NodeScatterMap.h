#pragma once

#include "dof_manager/ElementScatterMap.h"

namespace dof_manager
{
/// Stores which owned nodes needs to be sent to other ranks and which ghost
/// nodes to receive from other ranks.  Also, includes methods to exchange
/// node information needed for ghost layers between ranks.
/// For example:
/// \code{cpp}
/// NodeScatterMap scatter_map(mesh, element_scatter_map);
/// for (auto dest : scatter_map.destination_ranks_range())
/// {
///     auto dest_rank = dest.rank;
///     for (auto element_local_number : scatter_map.nodes_to_send_range(dest))
///     {
///         /// ...
///     }
/// }
/// for (auto source : scatter_map.source_ranks_range())
/// {
///     auto source_rank = source.rank;
///     for (auto element_local_number : scatter_map.nodes_to_receive_range(source))
///     {
///         /// ...
///     }
/// }
/// \endcode
struct NodeScatterMap
{
    /// Ranks to send each interval of nodes to.  Length (# of destination ranks)
    std::vector<int> destination_ranks;
    /// Contains the list of element indices to send to other ranks.
    std::vector<NodeLocalIndex> nodes_to_send;
    /// Contains the offsets for nodes_to_send for each destination rank to
    /// send to.  Length: (# of destination ranks + 1)
    /// Range of nodes to send for a particular rank is nodes_to_send[rank_offsets[i]]
    /// to nodes_to_send[rank_offsets[i + 1]] for destination rank i.
    std::vector<size_t> rank_send_offsets;

    /// Ranks to receive ghost nodes from nodes to.  Length (# of source ranks)
    std::vector<int> source_ranks;
    /// Contains the list of element indices to receive from other ranks.
    std::vector<NodeLocalIndex> nodes_to_receive;
    /// See rank_send_offsets for an explanation of the structure
    std::vector<size_t> rank_receive_offsets;


    NodeScatterMap(const finite_element::FiniteElementMesh& fe_mesh,
                   const ElementScatterMap& element_scatter_map);

    /// Provides a convenient way to iterate over the destination ranks with a
    /// range-based loop.  See the NodeScatterMap description for an example.
    detail::ScatterRankRange destination_ranks_range() const;

    /// Provides a convenient way to iterate over the nodes to send with a
    /// range-based loop.  See the NodeScatterMap description for an example.
    detail::ScatterNodeRange nodes_to_send_range(const detail::RankIndexValuePair& rank_pair) const;

    /// Provides a convenient way to iterate over the source ranks with a
    /// range-based loop.  See the NodeScatterMap description for an example.
    detail::ScatterRankRange source_ranks_range() const;

    /// Provides a convenient way to iterate over the nodes to receive with a
    /// range-based loop.  See the NodeScatterMap description for an example.
    detail::ScatterNodeRange nodes_to_receive_range(
      const detail::RankIndexValuePair& rank_pair) const;

  private:
    void build_maps(const finite_element::FiniteElementMesh& fe_mesh,
                    const ElementScatterMap& element_scatter_map);
};
}