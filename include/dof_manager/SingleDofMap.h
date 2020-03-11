#pragma once

#include "Defines.h"

namespace dof_manager
{
    /// \brief A helper class that provide convienient access to an single node or
    /// element's local to rank DoF map using references.
    class ToRankDofMap
    {
    public:
        ToRankDofMap(
            const RankDofIndex* const element_to_rank_map,
            const ElementDofIndex& n_dofs)
            : to_rank_map_(element_to_rank_map), n_dofs_(n_dofs)
        {
        }

        inline const RankDofIndex* begin() const
        {
            return to_rank_map_;
        }
        inline const RankDofIndex* end() const
        {
            return &to_rank_map_[n_dofs_];
        }
        inline RankDofIndex operator[](const ElementDofIndex i) const
        {
            return to_rank_map_[i];
        }
        inline ElementDofIndex size() const
        {
            return n_dofs_;
        }

    private:
        const RankDofIndex* const to_rank_map_;
        const ElementDofIndex& n_dofs_;
    };

    using NodeToRankDofMap = ToRankDofMap;
    using ElementToRankDofMap = ToRankDofMap;
}
