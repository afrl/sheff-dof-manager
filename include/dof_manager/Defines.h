#pragma once

#include "mesh/Defines.h"
#include "mesh/relations/ElementAdjacencyList.h"

#include <cstdint>

namespace dof_manager
{
    using GlobalDofIndex = std::uint32_t;
    using RankDofIndex = std::uint32_t;
    using ElementDofIndex = mesh::LocalIndex;
    using ElementLocalIndex = mesh::LocalIndex;
    using NodeLocalIndex = mesh::LocalIndex;
    using PartitionIndex = mesh::PartitionIndex;

    enum class DofType : char
    {
        OWNED,
        LOCAL_GHOST,
        EXCLUSIVE_GHOST
    };
}