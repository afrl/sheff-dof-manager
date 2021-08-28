#include "dof_manager/DofManager.h"

#include "dof_manager/DofRanges.h"
#include "dof_manager/detail/BasalFieldCompatibilityMap.h"
#include "dof_manager/detail/DofSorting.h"
#include "finite_element/FiniteElement.h"
#include "finite_element/hhe_primitives/EnrichmentTreeTraverser.h"
#include "finite_element/hhe_primitives/PrimitiveAlgorithms.h"
#include "utility/MPIUtils.h"

namespace dof_manager
{
//////////////////////////////////////////////////////////////////////////////////////////////////
// Helper methods

static DofMap enumeration_kernel(
  const finite_element::FiniteElementMesh& fe_mesh,
  const mesh::ElementAdjacencyList& element_adjacency,
  const ElementScatterMap& scatter_map,
  const detail::BasalFieldCompatibilityMap& basal_field_compatibility_map)
{
    auto local_rank = utility::MPIUtils::get_local_rank();
    // If this fails, then the partitioning algorithm is wrong, since the ownened elements
    // must be sorted to be at the front of the list.
    assert(fe_mesh.mesh.elements()[0].parent_partition() == local_rank);

    DofMap new_dof_map;

    // For now, assume each node has the same number of DoF's at the start of the analysis.
    // This algorithm will have to be generalized if this is not the case.
    auto n_dofs_per_point =
      finite_element::FiniteElement::get_n_dofs_per_point(*fe_mesh.layouts.front());

    constexpr RankDofIndex UNITIALIZED_DOF = std::numeric_limits<RankDofIndex>::max();
    new_dof_map.element_to_rank_maps.resize(fe_mesh.mesh.elements().size(), nullptr);
    new_dof_map.element_n_dofs.resize(fe_mesh.mesh.elements().size());
    new_dof_map.n_owned_elements = 0;
    for (auto fe : fe_mesh.finite_elements())
    {
        const auto& element_index = fe.element.local_index();
        const ElementDofIndex n_dofs_per_field =
          static_cast<ElementDofIndex>(fe.element.connectivity().size() * n_dofs_per_point);
        const finite_element::LeafIndex n_basal_fields = fe.layout.get_n_basal_fields();
        const ElementDofIndex n_dofs_for_element       = n_dofs_per_field * n_basal_fields;

        new_dof_map.element_to_rank_maps[element_index] = new RankDofIndex[n_dofs_for_element];
        new_dof_map.element_n_dofs[element_index]       = n_dofs_for_element;
        for (auto i : utility::range(n_dofs_for_element))
        {
            new_dof_map.element_to_rank_maps[element_index][i] = UNITIALIZED_DOF;
        }

        if (fe.element.parent_partition() == local_rank)
        {
            new_dof_map.n_owned_elements++;
        }
    }

    RankDofIndex owned_count = 0;
    RankDofIndex ghost_count = 0;
    for (auto fe : fe_mesh.finite_elements())
    {
        const auto& element_index = fe.element.local_index();
        const ElementDofIndex n_dofs_per_field =
          static_cast<ElementDofIndex>(fe.element.connectivity().size() * n_dofs_per_point);
        auto& element_dof_map = new_dof_map.element_to_rank_maps[element_index];


        auto& compatibilities = basal_field_compatibility_map.compatibilities[element_index];
        for (auto basal_field_index : utility::range(fe.layout.get_n_basal_fields()))
        {
            auto basal_field_dof_map = &element_dof_map[basal_field_index * n_dofs_per_field];

            // Carry over any DoFs from 1-to-1 compatible fields to ensure a continuous Galerkin
            // field across elements (except where enrichment introduce discontinuities or a front
            // creates a 1-to-many field relationship).
            //
            // Note: 1-to-many field relationships along a front are fixed by the fact that we
            // assume a closed cohesive zone along the front, but when the front grows, the DoFs
            // will be renumbered at the point to restore CG, and the front will introduce a
            // 1-to-many relationship further in the mesh where it grew to.
            for (auto& compatibility : compatibilities)
            {
                if (compatibility.this_basal_field_index == basal_field_index)
                {
                    auto neighbor_element =
                      fe_mesh.mesh.elements()[compatibility.neighbor_element_index];
                    ElementDofIndex n_dofs_per_neighbor_field = static_cast<ElementDofIndex>(
                      neighbor_element.connectivity().size() * n_dofs_per_point);

                    auto neighbor_field_dof_map =
                      &new_dof_map.element_to_rank_maps[neighbor_element.local_index()]
                                                       [compatibility.neighbor_basal_field_index *
                                                        n_dofs_per_neighbor_field];

                    auto shared_node_positions =
                      mesh::find_shared_node_positions(fe.element, neighbor_element);
                    for (auto& this_neighbor_node_pair : shared_node_positions)
                    {
                        const auto& this_node     = this_neighbor_node_pair.first;
                        const auto& neighbor_node = this_neighbor_node_pair.second;

                        auto this_dof_begin = &basal_field_dof_map[this_node * n_dofs_per_point];
                        auto neighbor_dof_begin =
                          &neighbor_field_dof_map[neighbor_node * n_dofs_per_point];

                        if (neighbor_dof_begin[0] != UNITIALIZED_DOF)
                        {
                            if (this_dof_begin[0] != UNITIALIZED_DOF &&
                                this_dof_begin[0] != neighbor_dof_begin[0])
                            {
                                detail::write_compatibility_graph("compatibility_graph",
                                                                  fe_mesh,
                                                                  basal_field_compatibility_map);
                                throw _UTIL_MSG_EXCEPT("DoF conflict found.");
                            }

                            for (auto i : utility::range(n_dofs_per_point))
                            {
                                this_dof_begin[i] = neighbor_dof_begin[i];
                            }
                        }
                    }
                }
            } // End loop over compatibilities

            // Any remaining DoFs that were not carried over from a compatible field, assign it a
            // new number.
            for (auto node_conn_index : utility::range(fe.element.connectivity().size()))
            {
                if (fe.element.node(node_conn_index).parent_partition() == local_rank)
                {
                    if (basal_field_dof_map[node_conn_index * n_dofs_per_point] == UNITIALIZED_DOF)
                    {
                        for (auto i : utility::range(n_dofs_per_point))
                        {
                            basal_field_dof_map[node_conn_index * n_dofs_per_point + i] =
                              owned_count++;
                        }
                    }
                }
                else
                {
                    if (basal_field_dof_map[node_conn_index * n_dofs_per_point] == UNITIALIZED_DOF)
                    {
                        for (auto i : utility::range(n_dofs_per_point))
                        {
                            basal_field_dof_map[node_conn_index * n_dofs_per_point + i] =
                              ghost_count++;
                        }
                    }
                }
            }
        }
    }

    const auto n_owned_dofs                = owned_count;
    const auto n_ghost_dofs                = ghost_count;
    const auto n_owned_dofs_on_lower_ranks = utility::MPIUtils::sum_of_lower_ranks(n_owned_dofs);
    new_dof_map.owned_global_dof_begin     = n_owned_dofs_on_lower_ranks;
    new_dof_map.owned_global_dof_end       = n_owned_dofs_on_lower_ranks + n_owned_dofs;

    // Go back over the element-to-rank DoF map and increment all ghost DoF's (rank numbering) to
    // start at the end of the owned DoF range
    for (auto fe : fe_mesh.finite_elements())
    {
        const auto& element_index = fe.element.local_index();
        const ElementDofIndex n_dofs_per_field =
          static_cast<ElementDofIndex>(fe.element.connectivity().size() * n_dofs_per_point);
        auto& element_dof_map = new_dof_map.element_to_rank_maps[element_index];

        for (auto basal_field_index : utility::range(fe.layout.get_n_basal_fields()))
        {
            auto basal_field_dof_map = &element_dof_map[basal_field_index * n_dofs_per_field];

            // Any remaining DoFs that were not carried over from a compatible field, assign it a
            // new number.
            for (auto node_conn_index : utility::range(fe.element.connectivity().size()))
            {
                if (fe.element.node(node_conn_index).parent_partition() != local_rank)
                {
                    for (auto i : utility::range(n_dofs_per_point))
                    {
                        basal_field_dof_map[node_conn_index * n_dofs_per_point + i] += n_owned_dofs;
                    }
                }
            }
        }
    }

    constexpr GlobalDofIndex GLOBAL_UNITIALIZED_DOF = std::numeric_limits<GlobalDofIndex>::max();
    new_dof_map.rank_to_global_map.resize(n_owned_dofs + n_ghost_dofs, GLOBAL_UNITIALIZED_DOF);
    for (auto i : utility::range(n_owned_dofs))
    {
        new_dof_map.rank_to_global_map[i] = new_dof_map.owned_global_dof_begin + i;
    }
    // At this point, the rank-to-global map is only half complete.  The owned DoFs are enumerated,
    // but MPI communication is needed for ghost DoF's

    // Build the buffer containing the element-to-global map for elements on this rank that appear
    // as ghost elements on other ranks.
    std::vector<GlobalDofIndex> send_buffer;
    std::vector<size_t> send_offsets = { 0 };
    for (auto dest_rank : scatter_map.destination_ranks_range())
    {
        for (auto local_element_index : scatter_map.elements_to_send_range(dest_rank))
        {
            // It is not neccessary to send the # of DoF's for the element, since the destination
            // rank can figure that out from the # of basal fields.  However, we will send it for
            // the sake of checking that ranks are consistent.
            send_buffer.push_back(new_dof_map.element_n_dofs[local_element_index]);

            // Note that this is also sending useless information for ghost DoF's (which are
            // equal to GLOBAL_UNINITIALIZED_DOF).  Ghost DoFs could be left off to reduce
            // communicated data.
            for (auto j : utility::range(new_dof_map.element_n_dofs[local_element_index]))
            {
                const auto& rank_index   = new_dof_map.element_to_rank_maps[local_element_index][j];
                const auto& global_index = new_dof_map.rank_to_global_map[rank_index];
                send_buffer.push_back(global_index);
            }
        }
        send_offsets.push_back(send_buffer.size()); // Marks the end of the data sent to "dest_rank"
    }

    // Exchange data with ghost layers
    std::vector<GlobalDofIndex> receive_buffer;
    std::vector<size_t> receive_offsets;
    utility::MPIUtils::exchange_arrays<GlobalDofIndex, size_t>(
      static_cast<int>(scatter_map.destination_ranks.size()),
      scatter_map.destination_ranks.data(),
      send_buffer.data(),
      send_offsets.data(),
      static_cast<int>(scatter_map.source_ranks.size()),
      scatter_map.source_ranks.data(),
      receive_buffer,
      receive_offsets);

    // Update the rank-to-global map for all of the ghost DoF's based on the element-to-global maps
    // received from other ranks.  Note: this hinges on the partitioning algorithm ensuring any
    // owned node appears in at least one owned element.
    for (auto source_rank : scatter_map.source_ranks_range())
    {
        auto curr_offset = receive_offsets[source_rank.index];
        for (auto local_element_index : scatter_map.elements_to_receive_range(source_rank))
        {
            assert(curr_offset < receive_offsets[source_rank.index + 1]);

            auto n_dofs_for_element = receive_buffer[curr_offset++];
            assert(new_dof_map.element_n_dofs[local_element_index] == n_dofs_for_element);

            auto fe = fe_mesh.get_finite_element(local_element_index);
            const ElementDofIndex n_dofs_per_field =
              static_cast<ElementDofIndex>(fe.element.connectivity().size() * n_dofs_per_point);
            auto& element_dof_map = new_dof_map.element_to_rank_maps[local_element_index];
            for (auto basal_field_index : utility::range(fe.layout.get_n_basal_fields()))
            {
                auto basal_field_dof_map = &element_dof_map[basal_field_index * n_dofs_per_field];

                // Any remaining DoFs that were not carried over from a compatible field, assign it
                // a new number.
                for (auto node_conn_index : utility::range(fe.element.connectivity().size()))
                {
                    if (fe.element.node(node_conn_index).parent_partition() != local_rank)
                    {
                        for (auto i : utility::range(n_dofs_per_point))
                        {
                            auto global_dof = receive_buffer[curr_offset++];
                            auto rank_dof =
                              basal_field_dof_map[node_conn_index * n_dofs_per_point + i];

                            assert(new_dof_map.rank_to_global_map[rank_dof] ==
                                     GLOBAL_UNITIALIZED_DOF ||
                                   new_dof_map.rank_to_global_map[rank_dof] == global_dof);

                            new_dof_map.rank_to_global_map[rank_dof] = global_dof;
                        }
                    }
                    else
                    {
                        curr_offset += n_dofs_per_point;
                    }
                }
            }
        }
        assert(curr_offset == receive_offsets[source_rank.index + 1]);
    }

#ifndef NDEBUG
    bool found_problem = false;
    for (auto fe : fe_mesh.finite_elements())
    {
        const auto& element_index = fe.element.local_index();
        const ElementDofIndex n_dofs_per_field =
          static_cast<ElementDofIndex>(fe.element.connectivity().size() * n_dofs_per_point);
        auto& element_dof_map = new_dof_map.element_to_rank_maps[element_index];
        for (auto basal_field_index : utility::range(fe.layout.get_n_basal_fields()))
        {
            auto basal_field_dof_map = &element_dof_map[basal_field_index * n_dofs_per_field];
            for (auto node_conn_index : utility::range(fe.element.connectivity().size()))
            {
                const auto& rank_dof   = basal_field_dof_map[node_conn_index * n_dofs_per_point];
                const auto& global_dof = new_dof_map.rank_to_global_map[rank_dof];
                if (global_dof < 0 || global_dof == GLOBAL_UNITIALIZED_DOF)
                {
                    found_problem = true;
                    std::cout << "Rank " << local_rank << " element " << fe.element.local_index()
                              << " (global_id " << fe.element.global_index() << ") basal_field "
                              << basal_field_index << " node " << (int)node_conn_index
                              << " (global_id " << fe.element.node(node_conn_index).global_index()
                              << ")" << std::endl;
                }
            }
        }
    }
    if (found_problem)
    {
        throw _UTIL_MSG_EXCEPT(
          "An error occurred during DoF enumeration.  Perhaps the partitioning is messed up?  See "
          "previous messages about which DoFs at which nodal positions were problems.");
    }
#endif

    detail::sort_intervals(new_dof_map);

    return new_dof_map;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// Definitions for DofManager methods

DofManager::DofManager() {}
DofManager::DofManager(DofManager&& other) noexcept
  : dof_map_(std::move(other.dof_map_))
  , basal_field_compatibility_map_(std::move(other.basal_field_compatibility_map_))
{
}
DofManager::~DofManager() {}

DofManager& DofManager::operator=(DofManager&& other)
{
    dof_map_                       = std::move(other.dof_map_);
    basal_field_compatibility_map_ = std::move(other.basal_field_compatibility_map_);
    return *this;
}

const detail::BasalFieldCompatibilityMap& DofManager::get_updated_basal_field_compatibility_map(
  const finite_element::FiniteElementMesh& fe_mesh,
  const mesh::ElementAdjacencyList& element_adjacency) const
{
    assert(basal_field_compatibility_map_);

    detail::update_compatibility_map(*basal_field_compatibility_map_, fe_mesh, element_adjacency);

    return *basal_field_compatibility_map_;
}

void DofManager::enumerate(const finite_element::FiniteElementMesh& fe_mesh,
                           const mesh::ElementAdjacencyList& element_adjacency,
                           const ElementScatterMap& scatter_map)
{
    auto local_rank = utility::MPIUtils::get_local_rank();
    // If this fails, then the partitioning algorithm is wrong, since the ownened elements
    // must be sorted to be at the front of the list.
    assert(fe_mesh.mesh.elements()[0].parent_partition() == local_rank);

    basal_field_compatibility_map_ = std::make_unique<detail::BasalFieldCompatibilityMap>(
      detail::build_initial_compatibity_map(fe_mesh, element_adjacency));

    // detail::write_compatibility_graph("initial_compatibility_graph",
    //                                  fe_mesh,
    //                                  *basal_field_compatibility_map_);

    dof_map_ =
      enumeration_kernel(fe_mesh, element_adjacency, scatter_map, *basal_field_compatibility_map_);
}

void DofManager::set_custom_enumeration(
  const ElementLocalIndex& n_owned_elements,
  const GlobalDofIndex& owned_begin,
  const GlobalDofIndex& owned_end,
  const std::vector<std::vector<RankDofIndex>>& element_to_rank_maps,
  const std::vector<GlobalDofIndex>& rank_to_global_map)
{
    dof_map_.rank_to_global_map = rank_to_global_map;
    dof_map_.element_n_dofs.resize(element_to_rank_maps.size(), 0);
    dof_map_.element_to_rank_maps.resize(element_to_rank_maps.size(), nullptr);
    for (auto i : utility::range(element_to_rank_maps.size()))
    {
        const auto n_dofs          = static_cast<RankDofIndex>(element_to_rank_maps[i].size());
        dof_map_.element_n_dofs[i] = n_dofs;
        dof_map_.element_to_rank_maps[i] = new RankDofIndex[n_dofs];
        std::copy(element_to_rank_maps[i].begin(),
                  element_to_rank_maps[i].end(),
                  dof_map_.element_to_rank_maps[i]);
    }
    dof_map_.owned_global_dof_begin = owned_begin;
    dof_map_.owned_global_dof_end   = owned_end;
    dof_map_.n_owned_dofs           = owned_end - owned_begin + 1;
    dof_map_.n_owned_elements       = n_owned_elements;

    detail::sort_intervals(dof_map_);
}

void DofManager::new_enrichment_callback(
  const finite_element::FiniteElementConstAccessor& fe_accessor,
  const finite_element::LeafIndex& basal_field_enriched)
{
    auto n_dofs_per_point = finite_element::FiniteElement::get_n_dofs_per_point(fe_accessor.layout);
    const ElementDofIndex n_dofs_per_field =
      static_cast<ElementDofIndex>(fe_accessor.element.connectivity().size() * n_dofs_per_point);
    const finite_element::LeafIndex n_basal_fields = fe_accessor.layout.get_n_basal_fields();

    const auto& element_index = fe_accessor.element.local_index();
    const auto& n_old_dofs    = dof_map_.element_n_dofs[element_index];
    const auto n_new_dofs     = n_old_dofs + n_dofs_per_field;

    RankDofIndex* new_dofs = new RankDofIndex[n_new_dofs];
    std::copy(dof_map_.element_to_rank_maps[element_index],
              &dof_map_.element_to_rank_maps[element_index][dof_map_.element_n_dofs[element_index]],
              new_dofs);
    auto enriched_field_start =
      &dof_map_.element_to_rank_maps[element_index][n_dofs_per_field * basal_field_enriched];
    for (auto i : utility::range(n_dofs_per_field))
    {
        new_dofs[n_old_dofs + i] = enriched_field_start[i];
    }

    delete dof_map_.element_to_rank_maps[element_index];
    dof_map_.element_to_rank_maps[element_index] = new_dofs;
    dof_map_.element_n_dofs[element_index]       = n_new_dofs;
}

std::vector<RankDofIndex> DofManager::renumber(const finite_element::FiniteElementMesh& fe_mesh,
                                               const mesh::ElementAdjacencyList& element_adjacency,
                                               const ElementScatterMap& scatter_map)
{
    assert(basal_field_compatibility_map_ != nullptr);

    detail::update_compatibility_map(*basal_field_compatibility_map_, fe_mesh, element_adjacency);

    detail::write_compatibility_graph("updated_compatibility_graph",
                                      fe_mesh,
                                      *basal_field_compatibility_map_);

    auto new_dof_map =
      enumeration_kernel(fe_mesh, element_adjacency, scatter_map, *basal_field_compatibility_map_);

    assert(new_dof_map.element_to_rank_maps.size() == dof_map_.element_to_rank_maps.size());
    std::vector<RankDofIndex> new_to_old_clone_map(new_dof_map.n_dofs_on_rank(),
                                                   std::numeric_limits<RankDofIndex>::max());
    for (auto element_index : utility::range(fe_mesh.mesh.elements().size()))
    {
        assert(new_dof_map.element_n_dofs[element_index] == dof_map_.element_n_dofs[element_index]);
        for (auto i : utility::range(dof_map_.element_n_dofs[element_index]))
        {
            new_to_old_clone_map[new_dof_map.element_to_rank_maps[element_index][i]] =
              dof_map_.element_to_rank_maps[element_index][i];
        }
    }
    for (auto& old_rank_index : new_to_old_clone_map)
    {
        assert(std::numeric_limits<RankDofIndex>::max());
    }

    dof_map_ = new_dof_map;

    return new_to_old_clone_map;
}

bool DofManager::requires_renumber(const finite_element::FiniteElementMesh& fe_mesh) const
{
    int renumber_flag = 0;
    for (auto element_index : utility::range(fe_mesh.mesh.elements().size()))
    {
        if (basal_field_compatibility_map_->n_basal_fields_at_last_update[element_index] !=
            fe_mesh.layouts[element_index]->get_n_basal_fields())
        {
            renumber_flag = 1;
            break;
        }
    }

    auto global_renumber_flag = utility::MPIUtils::sum_across_ranks(renumber_flag);
    return global_renumber_flag > 0;
}
}