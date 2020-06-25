#include "dof_manager/DofRanges.h"

#include "dof_manager/DofMap.h"

namespace dof_manager
{
    //////////////////////////////////////////////////////////////////////////////////////////////
    // Iterator Definitions

    detail::Iterator::Iterator(
        const DofMap& dof_map,
        RankDofIndex curr_rank_dof)
        : dof_map_(dof_map), curr_rank_dof_(curr_rank_dof)
    {
    }

    detail::Iterator& detail::Iterator::operator++()
    {
        curr_rank_dof_++;
        return *this;
    }

    detail::Iterator detail::Iterator::operator++(int)
    {
        detail::Iterator retval = *this;
        ++(*this);
        return retval;
    }

    bool detail::Iterator::operator==(detail::Iterator other) const
    {
        return curr_rank_dof_ == other.curr_rank_dof_;
    }

    bool detail::Iterator::operator!=(detail::Iterator other) const
    {
        return !(*this == other);
    }

    detail::Iterator::value_type detail::Iterator::operator*() const
    {
        return {
            curr_rank_dof_,
            dof_map_.rank_to_global_map[curr_rank_dof_],
            dof_map_.get_dof_type(curr_rank_dof_)
        };
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // GhostRange Definitions

    GhostRange::GhostRange(
        const DofMap& dof_map,
        const GhostRangeType type)
        : type_(type), dof_map_(dof_map)
    {
    }

    detail::Iterator GhostRange::begin() const
    {
        switch (type_)
        {
        case GhostRangeType::LOCAL:
        case GhostRangeType::ALL:
            return { dof_map_, dof_map_.n_owned_dofs };
        case GhostRangeType::EXCLUSIVE:
            return { dof_map_, dof_map_.n_local_dofs() };
        }
        throw _UTIL_MSG_EXCEPT_C("Ghost type range not implemented");
    }

    detail::Iterator GhostRange::end() const
    {
        switch (type_)
        {
        case GhostRangeType::LOCAL:
            return { dof_map_, dof_map_.n_local_dofs() };
        case GhostRangeType::ALL:
        case GhostRangeType::EXCLUSIVE:
            return { dof_map_, dof_map_.n_dofs_on_rank() };
        }
        throw _UTIL_MSG_EXCEPT_C("Ghost type range not implemented");
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // OwnedRange Definitions

    OwnedRange::OwnedRange(
        const DofMap& dof_map)
        : dof_map_(dof_map)
    {
    }

    detail::Iterator OwnedRange::begin() const
    {
        return { dof_map_, 0 };
    }

    detail::Iterator OwnedRange::end() const
    {
        return { dof_map_, dof_map_.n_owned_dofs };
    }
}


