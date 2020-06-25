#include "dof_manager/EnrichmentPhaseData.h"

namespace dof_manager
{
    void EnrichmentPhaseData::reset()
    {
        std::fill(element_enriched.begin(), element_enriched.end(), false);
        new_dof_to_old_rank_map.clear();
        dof_to_combine_map.clear();
        dof_combined_into_map.clear();
    }

    void EnrichmentPhaseData::combine_dofs(
        RankDofIndex lhs,
        RankDofIndex rhs)
    {
        process_dof_combination<RankDofIndex>(lhs, rhs, dof_to_combine_map, dof_combined_into_map);
    }


    const std::pair<bool, RankDofIndex> EnrichmentPhaseData::is_dof_combined_into_another(
        const RankDofIndex rank_dof_to_combine) const
    {
        auto find_iter = dof_to_combine_map.find(rank_dof_to_combine);
        if (find_iter == dof_to_combine_map.end())
        {
            return { false, 0 };
        }
        else
        {
            return { true, find_iter->second };
        }
    }
}