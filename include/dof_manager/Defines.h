#pragma once

#include "flex_mesh/common/Defines.h"

#include <cstdint>

namespace dof_manager
{
    using GlobalDofIndex = std::uint32_t;
    using RankDofIndex = std::uint32_t;
    using ElementDofIndex = flex_mesh::LocalIndex;
}