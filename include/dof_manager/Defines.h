#pragma once

#include "flex_mesh/common/Defines.h"
#include "flex_mesh/mesh/Mesh.h"
#include "flex_mesh/node/Node.h"
#include "flex_mesh/maps/AdjacencyLists.h"
#include "finite_element/FiniteElement.h"

#include <cstdint>

namespace dof_manager
{
    using GlobalDofIndex = std::uint32_t;
    using RankDofIndex = std::uint32_t;
    using ElementDofIndex = flex_mesh::LocalIndex;
    using ElementLocalIndex = flex_mesh::LocalIndex;
    using NodeLocalIndex = flex_mesh::LocalIndex;
    using PartitionIndex = flex_mesh::PartitionIndex;

    using MeshType = flex_mesh::Mesh<flex_mesh::Node, finite_element::FiniteElement>;
    using ElementAdjacencyList = flex_mesh::ElementAdjacencyList<MeshType>;

    enum class DofType : char
    {
        OWNED,
        LOCAL_GHOST,
        EXCLUSIVE_GHOST
    };
}