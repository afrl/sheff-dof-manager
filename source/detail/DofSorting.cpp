#include "dof_manager/detail/DofSorting.h"

#include "dof_manager/DofRanges.h"
#include "utility/MPIUtils.h"

namespace dof_manager
{
namespace detail
{

std::vector<std::size_t> sort_intervals(DofMap& dof_map)
{
    std::vector<DofType> dof_types_map(dof_map.rank_to_global_map.size(), DofType::EXCLUSIVE_GHOST);
    for (auto local_element_number : utility::range(dof_map.element_to_rank_maps.size()))
    {
        auto& element_to_rank_map = dof_map.element_to_rank_maps[local_element_number];
        for (auto i : utility::range(dof_map.element_n_dofs[local_element_number]))
        {
            auto& rank_index   = element_to_rank_map[i];
            auto& global_index = dof_map.rank_to_global_map[rank_index];
            if (global_index >= dof_map.owned_global_dof_begin &&
                global_index < dof_map.owned_global_dof_end)
            {
                dof_types_map[rank_index] = DofType::OWNED;
            }
            else if (local_element_number < dof_map.n_owned_elements)
            { // If the ghost appears in the owned element, then it is a local ghost
                dof_types_map[rank_index] = DofType::LOCAL_GHOST;
            }
            // else is not needed since it the value was initialized to EXCLUSIVE_GHOST
            // from the outset.
        }
    }

    // Update counts
    dof_map.n_owned_dofs = static_cast<RankDofIndex>(
      std::count_if(dof_types_map.begin(), dof_types_map.end(), [](const DofType& type) -> bool {
          return type == DofType::OWNED;
      }));
    dof_map.n_local_ghost_dofs = static_cast<RankDofIndex>(
      std::count_if(dof_types_map.begin(), dof_types_map.end(), [](const DofType& type) -> bool {
          return type == DofType::LOCAL_GHOST;
      }));
    dof_map.n_exclusive_ghost_dofs = static_cast<RankDofIndex>(dof_types_map.size()) -
                                     dof_map.n_owned_dofs - dof_map.n_local_ghost_dofs;


    // Sort the rank indices first by the DoF type then by the global index
    const auto& rank_to_global_map_ref = dof_map.rank_to_global_map;
    std::vector<std::size_t> backward_permutation_indices(dof_map.rank_to_global_map.size());
    std::iota(backward_permutation_indices.begin(), backward_permutation_indices.end(), 0);
    std::sort(backward_permutation_indices.begin(),
              backward_permutation_indices.end(),
              [&dof_types_map, &rank_to_global_map_ref](const std::size_t& lhs_index,
                                                        const std::size_t& rhs_index) -> bool {
                  return std::tie(dof_types_map[lhs_index], rank_to_global_map_ref[lhs_index]) <
                         std::tie(dof_types_map[rhs_index], rank_to_global_map_ref[rhs_index]);
              });

    std::vector<std::size_t> forward_permutation_indices(dof_map.rank_to_global_map.size());
    for (auto i : utility::indices(backward_permutation_indices))
    {
        forward_permutation_indices[backward_permutation_indices[i]] = i;
    }

    // Apply permutation
    apply_permutation(dof_map.rank_to_global_map, backward_permutation_indices);
    for (auto local_element_number : utility::range(dof_map.element_to_rank_maps.size()))
    {
        auto& element_to_rank_map = dof_map.element_to_rank_maps[local_element_number];
        for (auto i : utility::range(dof_map.element_n_dofs[local_element_number]))
        {
            element_to_rank_map[i] =
              static_cast<RankDofIndex>(forward_permutation_indices[element_to_rank_map[i]]);
        }
    }

    return backward_permutation_indices;
}

}
}