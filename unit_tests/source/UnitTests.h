#pragma once

#include "constitutive/field/deformation/linear_elasticity/LinearElasticProperties.h"
#include "dof_manager/DofManager.h"
#include "finite_element/FiniteElement.h"
#include "finite_element/FiniteElementMesh.h"
#include "finite_element/ModelSelectionHelper.h"
#include "finite_element/workspaces/ElementWorkspace.h"
#include "mesh/algorithms/Creation.h"
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
    finite_element::FiniteElementMesh fe_mesh;

    std::unique_ptr<finite_element::ElementWorkspace> workspace_ptr;

    UnitTestDataStructures(const std::array<int, 3> n_elements_along_axes)
      : fe_mesh(mesh::make_linear_grid(unit_linspace(n_elements_along_axes[0] + 1),
                                       unit_linspace(n_elements_along_axes[1] + 1),
                                       unit_linspace(n_elements_along_axes[2] + 1)))
    {
        constitutive::LinearElasticProperties isotropic_properties;
        isotropic_properties.set_isotropic_E_nu(1e9, 0.3);
        auto prop_ref =
          constitutive::FieldPropertiesCollection::singleton().add("isotropic_mat",
                                                                   std::move(isotropic_properties));


        workspace_ptr = std::make_unique<finite_element::ElementWorkspace>();
        for (auto& fe : fe_mesh.finite_elements())
        {
            finite_element::ModelSelectionHelper::get().assign_field_models(fe,
                                                                            "InfinitesimalStrain",
                                                                            prop_ref);
            finite_element::FiniteElement::allocate_isvs(fe.layout);
        }
    }
};

}