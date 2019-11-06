#pragma once

#include "dof_manager/DofMap.h"

#include "flex_mesh/algorithms/UnitElements.h"
#include "finite_element/elasticity/InfinitesimalStrain.h"
#include "constitutive/elasticity/LinearElasticProperties.h"
#include "constitutive/elasticity/LinearElasticityConstitutiveRelation.h"
#include "utility/MPIUtils.h"

#include <array>

using namespace dof_manager;

struct UnitTestDataStructures
{
    dof_manager::DofMap::MeshType mesh;
    constitutive::LinearElasticProperties properties;
    constitutive::LinearElasticityConstitutiveRelation constitutive_relation;
    std::unique_ptr<finite_element::ElementWorkspace> workspace_ptr;

    UnitTestDataStructures(
        const std::array<int, 3> n_elements_along_axes)
    {
        flex_mesh::get_unit_linear_hex(mesh);

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
