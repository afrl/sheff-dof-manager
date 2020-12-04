#pragma once

#include "dof_manager/Defines.h"

#include <map>
#include <set>
#include <vector>

namespace dof_manager
{

struct EnrichmentPhaseData
{
    /// A vector of bool's indicating if each owned element has been enriched this
    /// enrichment phase.  Vector is of size n_owned_elements_.
    std::vector<bool> element_enriched;
    /// \see DofMap::get_new_dof_to_old_rank_map
    std::vector<RankDofIndex> new_dof_to_old_rank_map;
    /// \see DofMap::combine_dofs
    /// A dof to be combined into another is a 1-to-1 relationship.
    std::map<RankDofIndex, RankDofIndex> dof_to_combine_map;
    /// The opposite of ep_dof_to_combine_map_, allowing looking up if a given dof is being
    /// combined into.  The key of this structure is the dof being combined into, and the value
    /// is a dof being combined.  There can be multiple dof's combined into a single dof, so
    /// a multimap is used.
    std::multimap<RankDofIndex, RankDofIndex> dof_combined_into_map;


    //////////////////////////////////////////////////////////////////////////////////////////
    // Query methods

    /// This is mostly used for unit-testing
    const std::pair<bool, RankDofIndex> is_dof_combined_into_another(
      const RankDofIndex rank_dof_to_combine) const;

    //////////////////////////////////////////////////////////////////////////////////////////
    // Modifier methods

    /// Mark that a dof should be combined into another during the next renumbering phase.
    /// This should be relatively rare, but sometimes, a compatibility of two enrichments
    /// will require combining dof's.  During the enrichment phase, either dof can be used
    /// interchangably, and during renumbering, one will be combined into another.
    ///
    /// Restriction: For now, rank_dof_to_combine and rank_dof_to_combine_into must both
    ///              be owned dof's. MPI communication will be needed for this to be relaxed.
    ///
    /// Proposal for combining dof's owned by different ranks.  Accumulate the combinations
    /// into a vector<pair<RankDofIndex, RankDofIndex>> if rank_dof_to_combine and/or
    /// rank_dof_to_combine_into is not owned.  After renumbering inserts new dof's then
    /// eliminates owned dof's from combinations, add a new step where dof's are combined across
    /// ranks.  Each rank will have to know about all combinations across rank boundaries to
    /// ensure they can keep their maps up to date, but at least only the combinations across
    /// ranks will be needed by all ranks (instead of all combinations internal to each rank).
    /// Unfortunately, a global-to-rank map might be needed since a rank doens't know about
    /// another rank's numbering.  But the global-to-rank map would only be needed in this
    /// step and only really needed for ghosts since there is an implicit global-to-rank map
    /// for owned dof's.
    void combine_dofs(RankDofIndex rank_dof_to_combine, RankDofIndex rank_dof_to_combine_into);

    /// Resets all enrichment data.  This should only be called by DofManager.renumber()
    void reset();
};

/// The kernel for dof combination, see EnrichmentPhaseData::combine_dofs().
/// Note: this is a template to allow reuse in the renumbering algorithm.
template<typename T>
static void process_dof_combination(
  T lhs,
  T rhs,
  std::map<RankDofIndex, RankDofIndex>& dof_to_combine_map,
  std::multimap<RankDofIndex, RankDofIndex>& dof_combined_into_map)
{
    const auto lhs_orig = lhs;
    const auto rhs_orig = rhs;

    // Make sure the dof being combined into is not already being combined into another.
    // If that is the case, then combine rank_dof_to_combine into what rank_dof_to_combine_into
    // is aleady being combined into.
    auto find_iter_lhs          = dof_to_combine_map.find(lhs);
    auto find_iter_rhs          = dof_to_combine_map.find(rhs);
    const bool lhs_is_dependent = find_iter_lhs != dof_to_combine_map.end();
    const bool rhs_is_dependent = find_iter_rhs != dof_to_combine_map.end();

    // If lhs is already dependent but rhs is not, then swich and substitute
    // existing relation.
    // Example:
    // Current set    Add                     Resulting set
    // A->B           C->D => D->C => D->B    A->B
    // C->B                                   C->B
    // E->D                                   E->B  (from back substitution)
    //                                        D->B  (new relation)
    if (lhs_is_dependent && !rhs_is_dependent)
    {
        // This combination is already taken care of (through the inverse)
        if (rhs == find_iter_lhs->second)
        {
            return;
        }

        std::swap(lhs, rhs);
        rhs = find_iter_lhs->second;
    }
    // If both are dependent then, create a new relation between the new independent
    // dof's (unless the two independent dof's are the same since the new relation is
    // already taken care of).
    // Example:
    // Current set    Add           Resulting set
    // A->B           A->D => B->E  A->E  (from back substitution)
    // C->B                         C->E  (from back substitution)
    // D->E                         D->E
    // F->E                         F->E
    //                              B->E  (new relation)
    else if (lhs_is_dependent && rhs_is_dependent)
    {
        if (find_iter_lhs->second == find_iter_rhs->second)
        {
            return;
        }
        else
        {
            lhs = find_iter_lhs->second;
            rhs = find_iter_rhs->second;
        }
    }
    else if (!lhs_is_dependent && rhs_is_dependent && lhs == find_iter_rhs->second)
    {
        return;
    }

    // Back substitution
    // The new relation still needs to be substituted into the existing relations
    // in case lhs appears as an independent dof in any existing relation.
    // Update relation combining something into lhs to now point to rhs.
    auto equals_range = dof_combined_into_map.equal_range(lhs);
    for (auto iter = equals_range.first; iter != equals_range.second; ++iter)
    {
        auto find_iter = dof_to_combine_map.find(iter->second);
        if (find_iter->first == rhs)
        {
            dof_to_combine_map.erase(find_iter);
        }
        else
        {
            find_iter->second = rhs;
        }
    }
    // Nothing is being combined into lhs anymore, so erase it from the map
    dof_combined_into_map.erase(lhs);

    // Update maps
    dof_to_combine_map.insert({ lhs, rhs });
    dof_combined_into_map.insert({ rhs, lhs });
}
}