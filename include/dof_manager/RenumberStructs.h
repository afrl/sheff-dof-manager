#pragma once

#include "dof_manager/Defines.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace dof_manager
{
    namespace detail
    {
        /// Helper function to permute a vector: https://devblogs.microsoft.com/oldnewthing/20170102-00/?p=95095
        /// \param v - vector to permute (in/out)
        /// \param backward_map - a vector filled with the old indices in the order that should form
        ///     the new vector, which is effectively a map from the new to the old indices.
        template<typename T, typename IndexType>
        void apply_permutation(
            std::vector<T>& v,
            const std::vector<IndexType>& backward_map)
        {
            std::vector<T> permuted_v;
            permuted_v.reserve(backward_map.size());
            for (IndexType i = 0; i < backward_map.size(); i++)
            {
                permuted_v.push_back(std::move(v[backward_map[i]]));
            }
            v = std::move(permuted_v);
        }

        template<typename IndexType>
        std::vector<IndexType> get_inverse_map(
            const std::vector<IndexType>& map)
        {
            std::vector<IndexType> inverse_map(map.size(), -1);
            for (auto i : utility::indices(map))
            {
                inverse_map[map[i]] = i;
            }
            assert(std::none_of(inverse_map.begin(), inverse_map.end(), [](const std::size_t& i) -> bool { return i == -1; }));
            return inverse_map;
        }
    }

    struct UpdateMapsLocalEnrichment
    {
        RankDofIndex n_new_dofs_on_this_rank;
        std::vector<int32_t> delta_n_dofs_by_rank;
        std::vector<RankDofIndex> post_enrichment_rank_to_new_rank_map;
        std::vector<RankDofIndex> new_rank_to_pre_enrichment_rank_map;
        std::unordered_map<RankDofIndex, RankDofIndex> owned_dofs_to_combine;
        std::unordered_set<RankDofIndex> owned_post_enrichment_dofs_combined_into;
    };

    struct DofRenumberingSource
    {
        enum SourceType
        {
            COPY_OLD_LOCAL_VALUE,
            COPY_NEW_GHOST_VALUE
        };

        SourceType source_type;
        RankDofIndex source_dof_rank_index;

        inline bool operator==(const DofRenumberingSource& other) const
        {
            return source_type == other.source_type &&
                source_dof_rank_index == other.source_dof_rank_index;
        }
    };

    struct RenumberingPhaseData
    {
        RankDofIndex pre_enrichment_n_owned_dofs;
        /// Contains the rank-to-global map from before the last enrichment phase
        std::vector<GlobalDofIndex> old_rank_to_global_map;
        /// Contains the map from the current rank DoF to a DoF that would contain a valid value.
        ///
        /// The source DoF could be a DoF that existed before the last enrichment phase, which
        /// will be indicated by source_type == COPY_LOCAL_VALUE.  For these, source_dof_rank_index
        /// refers to the old rank index to copy the value from.
        /// On the other hand, if source_type == COPY_GHOST_VALUE, then the value should be taken
        /// from the new value of a ghost DoF.
        ///
        /// When populating an updated global vector/matrix, set the values with COPY_LOCAL_VALUE
        /// first, then set COPY_GHOST_VALUE after communciating the ghost values.
        std::vector<DofRenumberingSource> owned_renumbering_sources;

        /// Permutes owned_renumbering_sources to a new ordering given permuation vector.
        ///
        /// \param backward_map - a vector filled with the old indices in the order that should form
        ///     the new vector, which is effectively a map from the new to the old indices.
        /// \param foreward_map - a vector filled with the new indices in the order that should form
        ///     the new vector, which is effectively a map from the old to the new indices.
        template<typename IndexType>
        void permute_owned_renumbering_sources(
            const IndexType new_n_owned_dofs,
            const std::vector<IndexType>& backward_map,
            const std::vector<IndexType>& forward_map)
        {
            std::vector<DofRenumberingSource> new_owned_renumbering_sources;
            new_owned_renumbering_sources.reserve(new_n_owned_dofs);
            for (IndexType i = 0; i < new_n_owned_dofs; i++)
            {
                new_owned_renumbering_sources.push_back(
                    std::move(owned_renumbering_sources[backward_map[i]]));
            }
            owned_renumbering_sources = std::move(new_owned_renumbering_sources);

            for (auto& v : owned_renumbering_sources)
            {
                if (v.source_type == DofRenumberingSource::COPY_NEW_GHOST_VALUE)
                {
                    v.source_dof_rank_index = forward_map[v.source_dof_rank_index];
                }
            }
        }
    };
}