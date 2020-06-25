#pragma once

#include "dof_manager/Defines.h"

namespace dof_manager
{
    namespace detail
    {
        struct RankIndexValuePair {
            std::size_t index;
            int rank;
        };

        /// Iterator class for DoF ranges
        class ScatterRankIterator
        {
        public:
            // Iterator traits
            using difference_type = RankIndexValuePair;
            using value_type = RankIndexValuePair;
            using pointer = const RankIndexValuePair*;
            using reference = const RankIndexValuePair&;
            using iterator_category = std::forward_iterator_tag;

            ScatterRankIterator(
                const std::vector<int>& ranks,
                const std::size_t curr_rank_index = 0);

            ScatterRankIterator& operator++();
            ScatterRankIterator operator++(int);
            bool operator==(ScatterRankIterator other) const;
            bool operator!=(ScatterRankIterator other) const;
            value_type operator*() const;

        private:
            const std::vector<int>& ranks_;
            std::size_t curr_rank_index_;
        };

        struct ScatterRankRange
        {
            const std::vector<int>& ranks;

            inline ScatterRankIterator begin() const { return { ranks, 0 }; }
            inline ScatterRankIterator end() const { return { ranks, ranks.size() }; }
        };

        struct ScatterElementRange
        {
            const ElementLocalIndex* begin_ptr;
            const ElementLocalIndex* end_ptr;

            inline const ElementLocalIndex* begin() const { return begin_ptr; }
            inline const ElementLocalIndex* end() const { return end_ptr; }
        };

        struct ScatterNodeRange
        {
            const NodeLocalIndex* begin_ptr;
            const NodeLocalIndex* end_ptr;

            inline const NodeLocalIndex* begin() const { return begin_ptr; }
            inline const NodeLocalIndex* end() const { return end_ptr; }
        };
    }
}