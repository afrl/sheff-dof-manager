#pragma once

#include "dof_manager/Defines.h"
#include "dof_manager/DofMap.h"

namespace dof_manager
{
namespace detail
{

/// Helper function to permute a vector:
/// https://devblogs.microsoft.com/oldnewthing/20170102-00/?p=95095 \param v - vector to permute
/// (in/out) \param backward_map - a vector filled with the old indices in the order that should
/// form the new vector, which is effectively a map from the new to the old indices.
template<typename T, typename IndexType>
void apply_permutation(std::vector<T>& v, const std::vector<IndexType>& backward_map)
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
std::vector<IndexType> get_inverse_map(const std::vector<IndexType>& map)
{
    std::vector<IndexType> inverse_map(map.size(), -1);
    for (auto i : utility::indices(map))
    {
        inverse_map[map[i]] = i;
    }
    assert(std::none_of(inverse_map.begin(), inverse_map.end(), [](const std::size_t& i) -> bool {
        return i == -1;
    }));
    return inverse_map;
}

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