#pragma once

#include "flex_mesh/mesh/Mesh.h"
#include "flex_mesh/node/Node.h"
#include "finite_element/FiniteElement.h"

#include <vector>

namespace dof_manager
{
    class DofMap
    {
    public:
        using MeshType = flex_mesh::Mesh<flex_mesh::Node, finite_element::FiniteElement>;
        using GlobalIndex = flex_mesh::GlobalIndex;
        using LocalIndex = flex_mesh::LocalIndex;

        /// The constructor will enumerate DoF's to the elements in the mesh provided.
        DofMap(const MeshType & mesh);

        /// Access the element to global DoF map.  The first index is the local element number
        /// and the second index is the global DoF number of the element-level DoF index.
        const GlobalIndex * const * const get_element_to_global_map() const;
        /// Access the number of DoF's each element.
        const LocalIndex * const get_element_num_dofs() const;
        /// Add a new element to the DoF map.  Returns the local element number.
        LocalIndex add_element(const std::vector<GlobalIndex>& element_global_dofs);
        /// Add new DoF's to an existing element, enriching the number DoF's for that element.
        void enrich_element(const LocalIndex & local_element_number,
            const std::vector<GlobalIndex>& new_dofs);

    private:
        const MeshType & mesh_;
        std::vector<GlobalIndex*> dof_map_;
        std::vector<LocalIndex> element_n_dofs_;

        void enumerate_dofs();
    };
}