#pragma once

#include "dof_manager/Defines.h"

namespace dof_manager
{
    struct DofMap;

    namespace detail
    {
        /// Iterator class for DoF ranges
        class Iterator
        {
        public:
            struct Value
            {
                RankDofIndex rank_dof;
                GlobalDofIndex global_dof;
                DofType type;
            };

            // Iterator traits
            using difference_type = RankDofIndex;
            using value_type = Value;
            using pointer = const Value*;
            using reference = const Value&;
            using iterator_category = std::forward_iterator_tag;

            Iterator(
                const DofMap& dof_map,
                RankDofIndex curr_rank_dof = 0);

            Iterator& operator++();
            Iterator operator++(int);
            bool operator==(Iterator other) const;
            bool operator!=(Iterator other) const;
            value_type operator*() const;

        private:
            const DofMap& dof_map_;
            RankDofIndex curr_rank_dof_;
        };
    }

    enum class GhostRangeType : char
    {
        LOCAL,
        EXCLUSIVE,
        ALL
    };

    /// This class provides convenient iteration over the various intervals of
    /// ghost DoF's.
    /// \code{cpp}
    /// for (auto ghost : GhostRange(scatter_map))
    /// {
    ///     // ghost.rank_dof contains the rank dof of the ghost
    ///     // ghost.global_dof contains the global dof of the ghost
    ///     // ghost.type contains the type of ghost
    /// }
    /// \endcode
    class GhostRange
    {
    public:
        GhostRange(
            const DofMap& scatter_map,
            const GhostRangeType type = GhostRangeType::ALL);
        detail::Iterator begin() const;
        detail::Iterator end() const;

    private:
        const GhostRangeType type_;
        const DofMap& dof_map_;
    };

    /// This class provides convenient iteration over the owned DoF's interval.
    class OwnedRange
    {
    public:
        OwnedRange(const DofMap& dof_map);
        detail::Iterator begin() const;
        detail::Iterator end() const;

    private:
        const DofMap& dof_map_;
    };
}