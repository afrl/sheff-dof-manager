#pragma once

#include "flex_mesh/mesh/Mesh.h"
#include "flex_mesh/node/Node.h"
#include "finite_element/FiniteElement.h"

#include <vector>

namespace dof_manager
{
    using GlobalIndex = flex_mesh::GlobalIndex;
    using LocalIndex = flex_mesh::LocalIndex;

    /// \brief A helper class that provide convienient access to an single element's local
    /// to global DoF map.
    class ElementDofMap
    {
    public:
        ElementDofMap(const GlobalIndex * const element_to_global_map,
            const LocalIndex & n_dofs);
        const GlobalIndex * const begin() const;
        const GlobalIndex * const end() const;
        const GlobalIndex operator[](const LocalIndex i) const;
        const LocalIndex size() const;

    private:
        const GlobalIndex * const element_to_global_map_;
        const LocalIndex & n_dofs_;
    };

    class DofMap
    {
    public:
        using MeshType = flex_mesh::Mesh<flex_mesh::Node, finite_element::FiniteElement>;

        /// The constructor will enumerate DoF's to the elements in the mesh provided.
        DofMap(const MeshType & mesh);

		///////////////
		// Accessors //
		///////////////

        /// Access the array of element to global DoF maps.
        /// The first index is the local element number and the second index is the global DoF
        /// number of the element-level DoF index.
        const GlobalIndex * const * const get_array_of_element_maps() const;
        /// Access the array of the number of DoF's each element.
        const LocalIndex * const get_array_of_element_n_dofs() const;
        /// Returns an object that wraps a single element's DoF map to provide convienient
        /// access iterators for loops.
        ElementDofMap get_element_map(const LocalIndex element_local_number) const;
		/// Access the node to global DoF map.  The first index is the local node number
		/// and the second index is the global DoF number of the node-level DoF index (ordered by
		/// twin then by field component).
		const GlobalIndex * const * const get_array_of_node_maps() const;
		/// Access the number of DoF's each node.
		const LocalIndex * const get_array_of_node_n_dofs() const;
        /// Returns the total number of DoFs (globally)
        const GlobalIndex get_n_global_dofs() const;
        /// Returns the number of DoFs on this rank
        const LocalIndex get_n_local_dofs() const;

		///////////////
		// Modifiers //
		///////////////

        /// Add a new element to the DoF map.  Returns the local element number.
        LocalIndex add_element(const std::vector<GlobalIndex>& element_global_dofs);
        /// Assign a new set of DoF's to an existing element, enriching the number DoF's for that element.
        void enrich_element(const LocalIndex & local_element_number,
            const std::vector<GlobalIndex>& replacement_dofs,
            GlobalIndex n_global_dofs); //TODO: stuff for updating node_map

    private:
        const MeshType & mesh_;
        std::vector<GlobalIndex*> element_dof_map_;
        std::vector<LocalIndex> element_n_dofs_;
		std::vector<GlobalIndex*> node_dof_map_;
		std::vector<LocalIndex> node_n_dofs_;
        GlobalIndex n_global_dofs_;
		LocalIndex n_local_dofs_;

        void enumerate_dofs();
    };
}