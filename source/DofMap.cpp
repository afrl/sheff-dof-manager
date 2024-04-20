#include "dof_manager/DofMap.h"

namespace dof_manager
{
DofMap::DofMap(const DofMap& other)
{
    *this = other;
}

DofMap::DofMap(DofMap&& other) noexcept
  : DofMap()
{
    *this = std::move(other);
}

DofMap::~DofMap()
{
    for (auto& map : element_to_rank_maps)
    {
        delete[] map;
    }
}

DofMap& DofMap::operator=(const DofMap& other)
{
    n_owned_elements = other.n_owned_elements;
    n_owned_dofs     = other.n_owned_dofs;
    n_local_ghost_dofs = other.n_local_ghost_dofs;
    n_exclusive_ghost_dofs = other.n_exclusive_ghost_dofs;
    owned_global_dof_begin = other.owned_global_dof_begin;
    owned_global_dof_end   = other.owned_global_dof_end;
    element_n_dofs         = other.element_n_dofs;
    rank_to_global_map     = other.rank_to_global_map;
    
    // Copy the element dof maps so they do not share the same pointers
    element_to_rank_maps.resize(other.element_to_rank_maps.size(), nullptr);
    for (auto i : utility::indices(element_to_rank_maps))
    {
        if (element_to_rank_maps[i] != nullptr)
        {
            delete[] element_to_rank_maps[i];
        }

        element_to_rank_maps[i] = new RankDofIndex[element_n_dofs[i]];
        std::copy(other.element_to_rank_maps[i],
                  other.element_to_rank_maps[i] + element_n_dofs[i],
                  element_to_rank_maps[i]);
    }

    return *this;
}

DofMap& DofMap::operator=(DofMap&& other) noexcept
{
    std::swap(n_owned_elements, other.n_owned_elements);
    std::swap(n_owned_dofs, other.n_owned_dofs);
    std::swap(n_local_ghost_dofs, other.n_local_ghost_dofs);
    std::swap(n_exclusive_ghost_dofs, other.n_exclusive_ghost_dofs);
    std::swap(owned_global_dof_begin, other.owned_global_dof_begin);
    std::swap(owned_global_dof_end, other.owned_global_dof_end);
    element_n_dofs.swap(other.element_n_dofs);
    element_to_rank_maps.swap(other.element_to_rank_maps);
    rank_to_global_map.swap(other.rank_to_global_map);

    return *this;
}

ElementToRankDofMap DofMap::get_element_map(const mesh::LocalIndex element_local_number) const
{
    return ElementToRankDofMap(element_to_rank_maps[element_local_number],
                               element_n_dofs[element_local_number]);
}


RankDofIndex DofMap::n_dofs_on_rank() const
{
    return static_cast<RankDofIndex>(rank_to_global_map.size());
}

RankDofIndex DofMap::n_local_dofs() const
{
    return n_owned_dofs + n_local_ghost_dofs;
}

DofType DofMap::get_dof_type(const RankDofIndex rank_dof) const
{
    if (rank_dof < n_owned_dofs)
    {
        return DofType::OWNED;
    }
    else if (rank_dof >= n_owned_dofs && rank_dof < n_owned_dofs + n_local_ghost_dofs)
    {
        return DofType::LOCAL_GHOST;
    }
    else if (rank_dof >= n_local_dofs() && rank_dof < n_dofs_on_rank())
    {
        return DofType::EXCLUSIVE_GHOST;
    }
    else
    {
        // Those outside of the ranges are assumed to be new DoF's during an enrichment phase,
        // which are owned.
        return DofType::OWNED;
    }
}
const std::string DofMap::str(const finite_element::FiniteElementMesh& fe_mesh) const
{
    std::stringstream ss;
    ss << "Element to Rank Map:" << std::endl;
    ss << element_n_dofs.size() << " elements" << std::endl;
    for (auto i : utility::range_as<mesh::LocalIndex>(element_n_dofs.size()))
    {
        ss << fe_mesh.mesh.elements()[i].global_index() << " : ";
        for (auto j : utility::range(element_n_dofs[i]))
        {
            ss << " " << element_to_rank_maps[i][j];
        }
        ss << std::endl;
    }
    ss << std::endl;

    ss << "Rank to Global Map:" << std::endl;
    ss << rank_to_global_map.size() << " DoFs on rank" << std::endl;
    for (auto i : utility::range(rank_to_global_map.size()))
    {
        ss << i << " -> " << rank_to_global_map[i] << std::endl;
    }
    ss << std::endl;

    return ss.str();
}
}