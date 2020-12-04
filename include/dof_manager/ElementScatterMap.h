#pragma once

#include "dof_manager/Defines.h"
#include "dof_manager/detail/ScatterMapHelpers.h"
#include "finite_element/FiniteElementMesh.h"

namespace dof_manager
{
/// Stores which owned elements needs to be sent to other ranks and which ghost
/// elements to receive from other ranks.  Also, includes methods to exchange
/// element information needed for ghost layers between ranks.
/// For example:
/// \code{cpp}
/// ElementScatterMap scatter_map(mesh, adjacency);
/// for (auto dest : scatter_map.destination_ranks_range())
/// {
///     auto dest_rank = dest.rank;
///     for (auto element_local_number : scatter_map.elements_to_send_range(dest))
///     {
///         /// ...
///     }
/// }
/// for (auto source : scatter_map.source_ranks_range())
/// {
///     auto source_rank = source.rank;
///     for (auto element_local_number : scatter_map.elements_to_receive_range(source))
///     {
///         /// ...
///     }
/// }
/// \endcode
struct ElementScatterMap
{
    /// Ranks to send each interval of elements to.  Length (# of destination ranks)
    std::vector<int> destination_ranks;
    /// Contains the list of element indices to send to other ranks.
    std::vector<ElementLocalIndex> elements_to_send;
    /// Contains the offsets for elements_to_send for each destination rank to
    /// send to.  Length: (# of destination ranks + 1)
    /// Range of elements to send for a particular rank is elements_to_send[rank_offsets[i]]
    /// to elements_to_send[rank_offsets[i + 1]] for destination rank i.
    std::vector<size_t> rank_send_offsets;

    /// Ranks to receive ghost elements from elements to.  Length (# of source ranks)
    std::vector<int> source_ranks;
    /// Contains the list of element indices to receive from other ranks.
    std::vector<ElementLocalIndex> elements_to_receive;
    /// See rank_send_offsets for an explanation of the structure
    std::vector<size_t> rank_receive_offsets;

    ElementScatterMap() {}

    void update(const finite_element::FiniteElementMesh& fe_mesh,
                const mesh::ElementAdjacencyList& adjacency);

    /// Provides a convenient way to iterate over the destination ranks with a
    /// range-based loop.  See the ElementScatterMap description for an example.
    detail::ScatterRankRange destination_ranks_range() const;

    /// Provides a convenient way to iterate over the elements to send with a
    /// range-based loop.  See the ElementScatterMap description for an example.
    detail::ScatterElementRange elements_to_send_range(
      const detail::RankIndexValuePair& rank_pair) const;

    /// Provides a convenient way to iterate over the source ranks with a
    /// range-based loop.  See the ElementScatterMap description for an example.
    detail::ScatterRankRange source_ranks_range() const;

    /// Provides a convenient way to iterate over the elements to receive with a
    /// range-based loop.  See the ElementScatterMap description for an example.
    detail::ScatterElementRange elements_to_receive_range(
      const detail::RankIndexValuePair& rank_pair) const;

    /// @brief Indicates whether update has been called at least once.  This does not indicate if the
    /// scatter map is still valid, and adding elements or dynamic load balancing to the mesh will
    /// indeed invalidate the scatter map.
    bool initialized() const;

  private:
    void build_receive_maps(const finite_element::FiniteElementMesh& fe_mesh);

    void build_send_maps(const finite_element::FiniteElementMesh& fe_mesh,
                         const mesh::ElementAdjacencyList& adjacency);
};
}