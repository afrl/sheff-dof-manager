#pragma once

#include "dof_manager/Defines.h"
#include "dof_manager/DofMap.h"
#include "dof_manager/ElementScatterMap.h"
#include "dof_manager/EnrichmentPhaseData.h"
#include "dof_manager/RenumberStructs.h"

#include <set>
#include <unordered_set>

namespace dof_manager
{
    // TODO rename to DofMap and rename the other class to DofManager
    class DofManager
    {
    public:

        //////////////////////////////////////////////////////////////////////////////////////////
        // Accessors

        const DofMap& get_dof_maps() const
        {
            return dof_map_;
        }
        const EnrichmentPhaseData& get_enrichment_data() const
        {
            return enrichment_data_;
        }
        DofMap get_dof_maps_copy() const
        {
            return dof_map_;
        }
        EnrichmentPhaseData get_enrichment_data_copy() const
        {
            return enrichment_data_;
        }

        //////////////////////////////////////////////////////////////////////////////////////////
        // Modifiers

        /// Enumerates the DoF's for the initial state before any enrichment occurs.  This is
        /// not allowed to be called multiple times.  It should only occur once near the beginning
        /// of an analysis.
        void enumerate(
            const MeshType& mesh,
            const ElementScatterMap& scatter_map);

        /// Sets the DoF enumeration state, allowing unit testing or a custom external initial
        /// DoF enumeration.  This is not expected to be used in BSAM++, except for testing.
        /// \param owned_begin - the first owned DoF of the global range by this rank.
        /// \param owned_end - the last owned DoF of the global range by this rank (inclusive range).
        void set_custom_enumeration(
            const ElementLocalIndex& n_owned_elements,
            const GlobalDofIndex& owned_begin,
            const GlobalDofIndex& owned_end,
            const std::vector<std::vector<RankDofIndex>>& element_to_rank_maps,
            const std::vector<GlobalDofIndex>& rank_to_global_map);

        /// Adds a new dof to the system adn returns the rank-level number, which
        /// can be used in the element-to-rank and node-to-element maps.
        ///
        /// \param original_sibling_dof - the existing dof that new_dof was created
        ///     to be a sibling to.  In the xfem code, when an element is enriched,
        ///     the old dof is used for one basal field, while a new dof is created
        ///     for the other basal field.  This parameter is the old dof.
        ///
        /// Note: because of this modifies counters and containers, enrichment of
        ///       elements MUST be sequential on each rank or a data race will ensue.
        ///       However, atomic operations could be implemented in the future.
        /// Note: if new DoF's need to be added for other reasons than XFEM, then
        ///       another version of add_new_dof should be created.
        RankDofIndex add_new_dof(
            const RankDofIndex original_sibling_dof);

        /// Assign a new set of DoF's to an existing element, enriching the number DoF's
        /// for that element.
        ///
        /// \param local_element_number - the rank-based local index of the element to enrich
        /// \param replacement_dofs - the ordered list of DoF's for all fields of the element
        ///        to replace the element-to-rank map with.
        ///
        /// Note: be sure to call add_new_dof() beforehand for any new DoF's that may
        ///       appear in replacement_dofs.
        void enrich_element(
            const flex_mesh::LocalIndex& local_element_number,
            const std::vector<RankDofIndex>& replacement_dofs);

        /// Mark that a dof should be combined into another during the next renumbering phase.
        /// Sometimes, a compatibility of two enrichments will require combining dof's.
        /// During the enrichment phase, either dof can be used interchangably, and during
        /// renumbering, one will be combined into another.
        void queue_dof_combination(
            RankDofIndex rank_dof_to_combine,
            RankDofIndex rank_dof_to_combine_into);

        /// Renumbering the rank and global numbering of all DoF's to account for:
        /// 1) New DoF's from enrichment
        /// 2) Combining DoF's
        /// 3) Moving DoF's between intervals
        ///
        /// Returns a RenumberingPhaseData object, which can be used by linear algebra containers
        /// to preserve the vectors in terms of the source DoF's or update preconditioners.
        RenumberingPhaseData renumber(
            const ElementScatterMap& element_scatter);

        //////////////////////////////////////////////////////////////////////////////////////////
        // Queries 

        /// Determines if a renumber is required (depends on if there were any enrichments
        /// or DoF combinations on any rank).
        bool requires_renumber() const;

    private:

        /// Stores the rank-to-global and element-to-rank maps along with some important
        /// counts that define various intervals.
        DofMap dof_map_;
        /// Stores information needed to track DoF modifications during enrichment phase.
        /// This is reset each time renumber() is called.
        EnrichmentPhaseData enrichment_data_;
    };

}