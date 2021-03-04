#pragma once

#include "dof_manager/Defines.h"
#include "dof_manager/DofMap.h"
#include "dof_manager/ElementScatterMap.h"
#include "finite_element/FiniteElementMesh.h"

#include <set>
#include <unordered_set>

namespace dof_manager
{
namespace detail
{
struct BasalFieldCompatibilityMap;
}

/// @brief  Creates degree of freedom (DoF) maps for a given state of a finite element analysis that
/// may include Hierarchical Heaviside Enrichment.
class DofManager
{
  public:
    DofManager();
    DofManager(DofManager&& other) noexcept;
    ~DofManager();

    DofManager& operator=(DofManager&& other);

    //////////////////////////////////////////////////////////////////////////////////////////
    // Accessors

    const DofMap& get_dof_maps() const
    {
        return dof_map_;
    }
    DofMap get_dof_maps_copy() const
    {
        return dof_map_;
    }

    /// TODO
    const detail::BasalFieldCompatibilityMap& get_updated_basal_field_compatibility_map(
      const finite_element::FiniteElementMesh& fe_mesh,
      const mesh::ElementAdjacencyList& element_adjacency) const;

    //////////////////////////////////////////////////////////////////////////////////////////
    // Modifiers

    /// Enumerates the DoF's for any state, even if enrichments exist.
    void enumerate(const finite_element::FiniteElementMesh& fe_mesh,
                   const mesh::ElementAdjacencyList& element_adjacency,
                   const ElementScatterMap& scatter_map);

    /// Sets the DoF enumeration state, allowing unit testing or a custom external initial
    /// DoF enumeration.  This is not expected to be used in BSAM++, except for testing.
    /// \param owned_begin - the first owned DoF of the global range by this rank.
    /// \param owned_end - the last owned DoF of the global range by this rank (inclusive range).
    void set_custom_enumeration(const ElementLocalIndex& n_owned_elements,
                                const GlobalDofIndex& owned_begin,
                                const GlobalDofIndex& owned_end,
                                const std::vector<std::vector<RankDofIndex>>& element_to_rank_maps,
                                const std::vector<GlobalDofIndex>& rank_to_global_map);

    /// Indicates to the DofManager that an element was enriched.  The element-to-rank map will be
    /// modified to ensure that global linear algebra vectors can still be accessed during the
    /// enrichment phase.  To accomplish this, the rank DoF's for the basal field that was enriched
    /// is simply appended (a copy) to the end of the element-to-rank map, thus depending on the
    /// convention that an enrichment reuses the enriched basal field and appends a new one.
    ///
    /// @important During enrichment phase, the DoF maps are in a state that allows the solution
    ///            vector to remain valid but also in a state that is very non-physical if the
    ///            solver were to be invoked.  Be sure to call renumber before solving after
    ///            new enrichments have been added.
    void new_enrichment_callback(const finite_element::FiniteElementConstAccessor& fe_accessor,
                                 const finite_element::LeafIndex& basal_field_enriched);

    /// Completely renumbers rank and global DoF indices to properly account for any new enrichments
    /// that have occurred. This first incrementally updates the inter-element basal field
    /// compatibility map, and then, second, uses the map to quickly create a new DoF enumeration.
    ///
    /// @returns A map from the new rank numbering (the index of the vector) to rank
    /// index of the DoF to clone the value from (the value of the vector) in order to retain a
    /// valid solution vector (and possibly preconditioners if they are very expensive to build).
    /// Heaviside enrichment splits a basal field into two, and though each of the two child fields
    /// have their own DoF's, it is still acknowledged that they were cloned from the original
    /// field's DoF's.
    std::vector<RankDofIndex> renumber(const finite_element::FiniteElementMesh& fe_mesh,
                                       const mesh::ElementAdjacencyList& element_adjacency,
                                       const ElementScatterMap& scatter_map);

    //////////////////////////////////////////////////////////////////////////////////////////
    // Queries

    /// Determines if a renumber is required (depends on if there were any enrichments on any rank).
    bool requires_renumber(const finite_element::FiniteElementMesh& fe_mesh) const;

    //////////////////////////////////////////////////////////////////////////////////////////////
    // Serailization methods

    template<typename SerializerType>
    void serialize(SerializerType& s) const
    {
        s(utility::named_ref("dof_map", dof_map_));
    }

    template<typename SerializerType>
    void deserialize(SerializerType& s)
    {
        s(dof_map_);
    }

  private:
    /// Stores the rank-to-global and element-to-rank maps along with some important
    /// counts that define various intervals.
    DofMap dof_map_;

    /// A map that indicates which basal fields in adjacent elements any given basal field is
    /// compatible with.  This can be incrementally updated when renumber() is called to account for
    /// new enrichments, and allows a quick enumeration of DoF's.
    std::unique_ptr<detail::BasalFieldCompatibilityMap> basal_field_compatibility_map_;
};

}