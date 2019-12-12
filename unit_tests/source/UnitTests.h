#pragma once

#include "dof_manager/DofMap.h"

#include "flex_mesh/algorithms/Creation.h"
#include "flex_mesh/algorithms/UnitElements.h"
#include "finite_element/elasticity/InfinitesimalStrain.h"
#include "constitutive/elasticity/LinearElasticProperties.h"
#include "constitutive/elasticity/LinearElasticityConstitutiveRelation.h"
#include "utility/MPIUtils.h"

#include <array>

namespace dof_manager
{
    static std::vector<double> unit_linspace(const int N)
    {
        std::vector<double> grid_offsets(N);
        for (auto i : utility::indices(grid_offsets))
        {
            grid_offsets[i] = double(i) / double(grid_offsets.size() - 1);
        }
        return grid_offsets;
    }

    struct UnitTestDataStructures
    {
        dof_manager::DofMap::MeshType mesh;
        constitutive::LinearElasticProperties properties;
        constitutive::LinearElasticityConstitutiveRelation constitutive_relation;
        std::unique_ptr<finite_element::ElementWorkspace> workspace_ptr;

        UnitTestDataStructures(
            const std::array<int, 3> n_elements_along_axes)
        {
            mesh = flex_mesh::make_grid<dof_manager::DofMap::MeshType>(
                unit_linspace(n_elements_along_axes[0] + 1),
                unit_linspace(n_elements_along_axes[1] + 1),
                unit_linspace(n_elements_along_axes[2] + 1),
                false);

            properties.set_isotropic_E_nu(1e9, 0.3);
            constitutive_relation.set_elastic_properties(properties);

            workspace_ptr = std::make_unique<finite_element::ElementWorkspace>();
            for (auto& e_ptr : mesh.elements())
            {
                auto formulation = std::make_unique<finite_element::InfinitesimalStrainFormulation>(e_ptr.get());
                e_ptr->set_formulation(std::move(formulation));
                e_ptr->set_constitutive_relation(&constitutive_relation);
                e_ptr->set_workspace(workspace_ptr.get());
                e_ptr->allocate_isvs();
            }
        }
    };

}