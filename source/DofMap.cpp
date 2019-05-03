#include "dof_manager/DofMap.h"

#include "finite_element/BaseFormulation.h"
#include "finite_element/FiniteElement.h"

#include "utility/MPIUtils.h"
#include "utility/Range.h"

namespace dof_manager
{
    ///////////////////////////
    // ElementDofMap Methods //
    ///////////////////////////

    ElementDofMap::ElementDofMap(const GlobalIndex * const & element_to_global_map, const LocalIndex & n_dofs)
        : element_to_global_map_(element_to_global_map), n_dofs_(n_dofs)
    {
    }

    const GlobalIndex * const ElementDofMap::begin() const
    {
        return element_to_global_map_;
    }

    const GlobalIndex * const ElementDofMap::end() const
    {
        return &element_to_global_map_[n_dofs_];
    }

    const GlobalIndex ElementDofMap::operator[](const LocalIndex i) const
    {
        return element_to_global_map_[i];
    }

    const LocalIndex ElementDofMap::size() const
    {
        return n_dofs_;
    }

    ////////////////////
    // DofMap Methods //
    ////////////////////

    DofMap::DofMap(const MeshType & mesh)
        : mesh_(mesh), n_global_dofs_(0), n_local_dofs_(0)
    {
        enumerate_dofs();
    }

    const GlobalIndex * const * const DofMap::get_array_of_element_maps() const
    {
        return &element_dof_map_[0];
    }

    const LocalIndex * const DofMap::get_array_of_element_n_dofs() const
    {
        return &element_n_dofs_[0];
    }

    ElementDofMap DofMap::get_element_map(const LocalIndex element_local_number) const
    {
        return ElementDofMap(element_dof_map_[element_local_number],
            element_n_dofs_[element_local_number]);
    }

	const GlobalIndex * const * const DofMap::get_array_of_node_maps() const
	{
		return &node_dof_map_[0];
	}

	const LocalIndex * const DofMap::get_array_of_node_n_dofs() const
	{
		return &node_n_dofs_[0];
	}

    const GlobalIndex DofMap::get_n_global_dofs() const
    {
        return n_global_dofs_;
    }

    const GlobalIndex DofMap::get_n_local_dofs() const
    {
        return n_local_dofs_;
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
            element_dof_map_.resize(mesh_.elements().size(), nullptr);
            element_n_dofs_.resize(mesh_.elements().size(), 0);

			node_dof_map_.resize(mesh_.nodes().size(), nullptr);
			node_n_dofs_.resize(mesh_.nodes().size(), dofs_per_node);
			for (auto& n_ptr : mesh_.nodes())
			{
				GlobalIndex * & node_dof_map = node_dof_map_[n_ptr->get_local_number()];
				node_dof_map = new GlobalIndex[dofs_per_node];
				for (LocalIndex i = 0; i < dofs_per_node; i++)
				{
					node_dof_map[i] = n_ptr->get_global_number() * dofs_per_node + i;
				}
			}

            for (auto& e_ptr : mesh_.elements())
            {
                const auto n_dofs_for_element = e_ptr->get_num_nodes() * dofs_per_node;
				element_dof_map_[e_ptr->get_local_number()] = new GlobalIndex[n_dofs_for_element];
                element_n_dofs_[e_ptr->get_local_number()] = n_dofs_for_element;

                GlobalIndex * & element_dof_map = element_dof_map_[e_ptr->get_local_number()];
                LocalIndex n_index = 0;
                for (auto& n_ptr : *e_ptr)
                {
                    for (LocalIndex i = 0; i < dofs_per_node; i++)
                    {
                        element_dof_map[n_index * dofs_per_node + i] = n_ptr->get_global_number() * dofs_per_node + i;
                    }
                    n_index++;
                }
            }
            n_global_dofs_ = static_cast<GlobalIndex>(mesh_.nodes().size()) * dofs_per_node;
            n_local_dofs_ = static_cast<LocalIndex>(n_global_dofs_);
        }
    }

    void DofMap::enrich_element(
            const LocalIndex & local_element_number,
            const std::vector<GlobalIndex>& replacement_dofs)
    {
        auto n_old_dofs = element_n_dofs_[local_element_number];
        auto n_new_dofs = replacement_dofs.size();
        auto n_added_dofs = n_new_dofs - n_old_dofs;

        n_global_dofs_ += n_added_dofs;
        n_local_dofs_ += n_added_dofs; //Do we need this?

        delete element_dof_map_[local_element_number];
        GlobalIndex *new_dofs = new GlobalIndex[n_new_dofs];
        element_dof_map_[local_element_number] = new_dofs;
        std::copy(replacement_dofs.begin(), replacement_dofs.end(), new_dofs);

        element_n_dofs_[local_element_number] = n_new_dofs;

        //TODO: Update node map?

    }
}
