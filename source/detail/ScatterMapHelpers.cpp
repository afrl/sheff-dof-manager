#include "dof_manager/detail/ScatterMapHelpers.h"

#include "utility/MPIUtils.h"

namespace dof_manager
{
    namespace detail
    {
        ScatterRankIterator::ScatterRankIterator(
            const std::vector<int>& ranks,
            const std::size_t curr_rank_index)
            : ranks_(ranks), curr_rank_index_(curr_rank_index)
        {
        }

        ScatterRankIterator& ScatterRankIterator::operator++()
        {
            curr_rank_index_++;
            return *this;
        }

        ScatterRankIterator ScatterRankIterator::operator++(int)
        {
            detail::ScatterRankIterator retval = *this;
            ++(*this);
            return retval;
        }

        bool ScatterRankIterator::operator==(ScatterRankIterator other) const
        {
            return curr_rank_index_ == other.curr_rank_index_;
        }

        bool ScatterRankIterator::operator!=(ScatterRankIterator other) const
        {
            return curr_rank_index_ != other.curr_rank_index_;
        }

        ScatterRankIterator::value_type ScatterRankIterator::operator*() const
        {
            return {
                curr_rank_index_,
                ranks_[curr_rank_index_]
            };
        }
    }
}