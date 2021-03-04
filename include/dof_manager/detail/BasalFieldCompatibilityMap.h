#pragma once

#include "finite_element/Defines.h"
#include "finite_element/FiniteElementMesh.h"
#include "mesh/Defines.h"
#include "mesh/relations/ElementAdjacencyList.h"

namespace dof_manager
{
namespace detail
{

/// @brief Tracks a single compatibility between one basal field (the parent element is implicit
/// within BasalFieldCompatibilityMap) and a basal field in a neighboring element.
struct BasalFieldCompatibility
{
    const finite_element::LeafIndex this_basal_field_index;
    const mesh::LocalIndex neighbor_element_index;
    const finite_element::LeafIndex neighbor_basal_field_index;
};

/// @brief Stores all 1-to-1 compatibility relationships between basal fields across elements, which
/// is essential to enumerating DoF's.
struct BasalFieldCompatibilityMap
{
    // Note: the size type needs to be much larger than finite_element::LeafIndex (int8_t)
    std::vector<utility::DynamicArray<BasalFieldCompatibility, int16_t>> compatibilities;
    std::vector<utility::DynamicArray<BasalFieldCompatibility, int16_t>> incompatibilities;
    // Stores the # of basal fields when the map was last updated, allowing the update algorithm to
    // detect what entries need to be updated.
    std::vector<finite_element::LeafIndex> n_basal_fields_at_last_update;
};

/// @brief Builds an initial inter-element basal field compatibility for the current state.
BasalFieldCompatibilityMap build_initial_compatibity_map(
  const finite_element::FiniteElementMesh& fe_mesh,
  const mesh::ElementAdjacencyList& element_adjacency);

/// @brief Updates a compatibility map based on any new enrichments that have occurred since the
///        last update/initialization.
/// @returns A structure containing a map that indicates which basal field any basal field in
///          recently enriched elements should clone DoF's from for the purpose of updating
///          persistent linear algebra structures (like the solution vector).  Note: the empty
///          is simply empty for any elements that were not modified.
void update_compatibility_map(BasalFieldCompatibilityMap& map,
                              const finite_element::FiniteElementMesh& fe_mesh,
                              const mesh::ElementAdjacencyList& element_adjacency);

/// TODO
bool compatibility_exists(BasalFieldCompatibilityMap& map,
                          const mesh::LocalIndex element_index_1,
                          const finite_element::LeafIndex basal_field_index_1,
                          const mesh::LocalIndex element_index_2,
                          const finite_element::LeafIndex basal_field_index_2);

/// TODO
bool incompatibility_exists(BasalFieldCompatibilityMap& map,
                            const mesh::LocalIndex element_index_1,
                            const finite_element::LeafIndex basal_field_index_1,
                            const mesh::LocalIndex element_index_2,
                            const finite_element::LeafIndex basal_field_index_2);

/// TODO
void write_compatibility_graph(const std::string& filestem,
                               const finite_element::FiniteElementMesh& fe_mesh,
                               const BasalFieldCompatibilityMap& map);

/// TODO
void write_incompatibility_graph(const std::string& filestem,
                                 const finite_element::FiniteElementMesh& fe_mesh,
                                 const BasalFieldCompatibilityMap& map);

}
}