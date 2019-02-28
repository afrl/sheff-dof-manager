#include "dof_manager/DofMap.h"

#include "finite_element/BaseFormulation.h"
#include "finite_element/FiniteElement.h"

#include "utility/MPIUtils.h"

namespace dof_manager
{
    DofMap::DofMap(const MeshType & mesh)
        : mesh_(mesh)
    {
        enumerate_dofs();
    }

    const DofMap::GlobalIndex * const * const DofMap::get_element_to_global_map() const
    {
        return &dof_map_[0];
    }

    const DofMap::LocalIndex * const DofMap::get_element_num_dofs() const
    {
        return &element_n_dofs_[0];
    }

    void DofMap::enumerate_dofs()
    {
        const bool is_distributed = utility::MPIUtils::is_distributed();
        if (is_distributed)
        {
            throw _UTIL_MSG_EXCEPT_C("TODO implement distributed DoF enumeration.");
        }
        else
        {
            // For now, assume each node has the same number of DoF's at the start of the analysis.
            // This algorithm will have to be generalized if this is not the case.
            auto dofs_per_node = mesh_.elements().front()->get_formulation_const().get_n_dofs_per_node();
            dof_map_.resize(mesh_.elements().size(), nullptr);
            element_n_dofs_.resize(mesh_.elements().size(), 0);
            for (auto& e_ptr : mesh_.elements())
            {
                const auto n_dofs_for_element = e_ptr->get_num_nodes() * dofs_per_node;
                dof_map_[e_ptr->get_local_number()] = new GlobalIndex[n_dofs_for_element];
                element_n_dofs_[e_ptr->get_local_number()] = n_dofs_for_element;

                GlobalIndex * & element_dof_map = dof_map_[e_ptr->get_local_number()];
                LocalIndex n_index = 0;
                for (auto& n_ptr : *e_ptr)
                {
                    for (auto i = 0; i < dofs_per_node; i++)
                    {
                        element_dof_map[n_index * dofs_per_node + i] = n_ptr->get_global_number() * dofs_per_node + i;
                    }
                    n_index++;
                }
            }
        }
    }
}
