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
    namespace detail
    {
        /// Enumerates the DoF's for the initial state before any enrichment occurs.  This is
        /// not allowed to be called multiple times.  It should only occur once near the beginning
        /// of an analysis.
        /// \param mesh - the mesh for the respective partition of this rank, this may or may not
        ///     include ghost elements.
        /// \param dof_map - the DoF map to fill with the enumeration.
        void create_initial_enumeration(
            const MeshType& mesh,
            const ElementScatterMap& scatter_map,
            DofMap& dof_map);

        /// Creates a old-to-new and new-to-old rank DoF maps for incorporating the new DoF's
        /// on this rank into the owned interval and combining any DoF's that are both owned.
        /// Removes DoF combinations that are accounted through the maps from enrichment_data.
        UpdateMapsLocalEnrichment create_maps_for_local_enrichment_and_combinations(
            const DofMap& dof_map,
            EnrichmentPhaseData& enrichment_data);

        /// This must be called before exchange_new_dofs() since the global numbering must be
        /// consistent across ranks before information can be exchanged.
        RenumberingPhaseData update_owned_global_numbering(
            const UpdateMapsLocalEnrichment& update_maps,
            DofMap& dof_map);

        /// TODO
        void update_ghost_pre_enrichment_global_numbering(
            const ElementScatterMap& element_scatter,
            const RenumberingPhaseData& renumbering_data,
            UpdateMapsLocalEnrichment& update_maps,
            DofMap& dof_map);

        /// TODO
        void exchange_new_exclusive_ghost_dofs(
            const ElementScatterMap& element_scatter,
            const RenumberingPhaseData& renumbering_data,
            const UpdateMapsLocalEnrichment& update_maps,
            DofMap& dof_map);

        /// After update_global_numbering() has been called, the rank numbering has been modified
        /// for the rank-to-global map, so this must be called before exchange_new_dofs() so a
        /// element-to-global map can be created for enriched elements, which will be exchanged
        /// with ghost ranks.
        void update_element_to_rank_map(
            const UpdateMapsLocalEnrichment& update_maps,
            DofMap& dof_map);

        /// TODO
        std::unordered_map<GlobalDofIndex, RankDofIndex> get_global_to_rank_map(
            const DofMap& dof_map);

        // TODO update doc string
        /// Updates any owned elements that were enriched on the other ranks that have the element
        /// as a ghost, and also updates this rank's DoF maps for any enriched ghost elements.
        /// This also adds any DoF's that are new to this rank to the exclusive ghost range.
        ///
        /// Note: Adds new DoF's from other ranks to the exclusive ghost interval.  This means that
        ///       update_maps.new_rank_to_old_rank_map is only valid for the local interval now as it
        ///       does not contain any of the new exclusive ghost DoF's.
        void exchange_enriched_ghost_elements(
            const EnrichmentPhaseData& enrichment_data,
            const ElementScatterMap& element_scatter,
            const std::unordered_map<GlobalDofIndex, RankDofIndex>& global_to_rank_map,
            DofMap& dof_map);


        /// Gather all of these to the root, process combinations to ensure uniqueness, broadcast
        /// back out.  Each rank receives all of the inter-rank combinations.
        void resolve_inter_rank_combinations(
            const EnrichmentPhaseData& enrichment_data,
            const std::unordered_map<GlobalDofIndex, RankDofIndex>& global_to_rank_map,
            const UpdateMapsLocalEnrichment& update_maps,
            RenumberingPhaseData& renumbering_data,
            DofMap& dof_map);

        /// Sort rank numbering to restore each of the three intervals and sort within each
        /// interval by the global numbers.
        /// This modifies:
        ///  - rank_to_global_map
        ///  - element_to_rank_maps
        /// Returns the backward permutation in case other vectors based on rank numbering need
        /// to be updated, this is essentially a map from the indices after sorting to the indices
        /// before sorting.
        std::vector<std::size_t> sort_intervals(DofMap& dof_map);
    }
}