#pragma once

#include "dof_manager/Defines.h"
#include "finite_element/FiniteElementMesh.h"
#include "utility/SliceList.h"

#include <vector>

namespace dof_manager
{
using ElementIndex = uint16_t;
using RankIndex    = uint32_t;
using GlobalIndex  = uint32_t;

/// @brief A helper class that provide convienient access to an single node or
/// element's local to rank DoF map using references.
class ToRankDofMap
{
  public:
    ToRankDofMap(const RankDofIndex* const element_to_rank_map, const ElementDofIndex& n_dofs)
      : to_rank_map_(element_to_rank_map)
      , n_dofs_(n_dofs)
    {
    }

    inline const RankDofIndex* begin() const
    {
        return to_rank_map_;
    }
    inline const RankDofIndex* end() const
    {
        return &to_rank_map_[n_dofs_];
    }
    inline RankDofIndex operator[](const ElementDofIndex i) const
    {
        return to_rank_map_[i];
    }
    inline ElementDofIndex size() const
    {
        return n_dofs_;
    }

  private:
    const RankDofIndex* const to_rank_map_;
    const ElementDofIndex& n_dofs_;
};

using NodeToRankDofMap    = ToRankDofMap;
using ElementToRankDofMap = ToRankDofMap;

/// @brief Stores all neccessary maps for degrees of freedom (DoFs)
struct DofMap
{
    /// The number of elements owned by this rank
    mesh::LocalIndex n_owned_elements;

    /// The number of DoF's owned by this rank (excludes ghosts).
    /// Due to the global number scheme, this is equivalent to:
    /// owned_global_dof_end_ - owned_global_dof_begin_
    /// Note: during the enrichment phase, this will be incorrect as new DoF's
    //        do not have global numbers until the renumbering phase.
    RankDofIndex n_owned_dofs;
    /// The number of DoF's appearing in owned element DoF maps but are owned by other ranks
    RankDofIndex n_local_ghost_dofs;
    /// The number of DoF's that ONLY appear in ghost elements and are owned by other ranks
    RankDofIndex n_exclusive_ghost_dofs;

    /// Since global numbering scheme will maintain that the DoF's owned by this rank
    /// lie within a continuous interval (with each rank's interval sorted by rank),
    /// this is the beginning of the range (inclusive).
    /// Note: during the enrichment phase, this will be incorrect as new DoF's
    //        do not have global numbers until the renumbering phase.
    GlobalDofIndex owned_global_dof_begin;
    /// The end of the global owned range (inclusive). \see owned_global_dof_begin_
    /// Note: during the enrichment phase, this will be incorrect as new DoF's
    //        do not have global numbers until the renumbering phase.
    GlobalDofIndex owned_global_dof_end;

    /// The number of DoF's in each element, corresponds to element_to_rank_maps_
    std::vector<ElementDofIndex> element_n_dofs;

    /// Contains the rank DoF map for each element.  Owned elements are first, followed by ghost
    /// elements.  The size of the vector is the number of owned + ghosts elements.
    std::vector<RankDofIndex*> element_to_rank_maps;

    /// The map that permutes the rank-based DoF number to global number.
    /// It will be of size n_all_rank_dofs_.
    ///
    /// The global numbering will always ensure:
    /// 1) All DoF's owned by a rank is a continuous interval of integers
    /// 2) The intervals owned by each rank are sorted by rank
    /// 3) As new DoF's are added to a ranks interval, they are appended to
    ///    the end of the interval of that rank (resulting in the global DoF
    ///    numbers incrementing by the number of new DoF's for all higher ranks)
    /// Because these properties, the global numbering scheme is suitable for
    /// a PETSc global numbering scheme without modification.
    ///
    /// The rank numbering orders DoF's into 5 intervals:
    /// - owned DoF's
    /// - local ghost DoF's
    /// - exclusive ghost DoF's
    std::vector<GlobalDofIndex> rank_to_global_map;


    DofMap();
    DofMap(const DofMap& other);
    DofMap(DofMap&& other) noexcept;
    ~DofMap();

    DofMap& operator=(const DofMap& other);
    DofMap& operator=(DofMap&& other) noexcept;

    //////////////////////////////////////////////////////////////////////////////////////////
    // Queries

    /// Returns an object that wraps a single element's DoF map to provide convenient
    /// access iterators for loops.
    ElementToRankDofMap get_element_map(const mesh::LocalIndex element_local_number) const;


    /// The number of DoF's appearing in the DoF maps of all elements on this rank
    /// (owned and ghost).
    RankDofIndex n_dofs_on_rank() const;

    /// The number of DoF's appearing in the DoF maps of owned elements.
    /// This excludes DoF's that only appear in ghost elements.
    RankDofIndex n_local_dofs() const;

    /// Returns the type of DoF, which depends on which interval the DoF falls in.
    DofType get_dof_type(const RankDofIndex rank_dof) const;

    const std::string str(const finite_element::FiniteElementMesh& fe_mesh) const;


    //////////////////////////////////////////////////////////////////////////////////////////////
    // Serailization methods

    template<typename SerializerType>
    void serialize(SerializerType& s) const
    {
        mesh::LocalIndex n_local_elements = static_cast<mesh::LocalIndex>(element_n_dofs.size());

        s(utility::named_ref("n_owned_elements", n_owned_elements),
          utility::named_ref("n_local_elements", n_local_elements),
          utility::named_ref("n_owned_dofs", n_owned_dofs),
          utility::named_ref("n_local_ghost_dofs", n_local_ghost_dofs),
          utility::named_ref("n_exclusive_ghost_dofs", n_exclusive_ghost_dofs),
          utility::named_ref("owned_global_dof_begin", owned_global_dof_begin),
          utility::named_ref("owned_global_dof_end", owned_global_dof_end));

        s(utility::named_value("element_n_dofs", utility::slice_c(element_n_dofs)),
          utility::named_value("element_to_rank_maps",
                               utility::slice_list(element_to_rank_maps, element_n_dofs)),
          utility::named_value("rank_to_global_map", utility::slice_c(rank_to_global_map)));
    }

    template<typename SerializerType>
    void deserialize(SerializerType& s)
    {
        mesh::LocalIndex n_local_elements;
        s(n_owned_elements,
          n_local_elements,
          n_owned_dofs,
          n_local_ghost_dofs,
          n_exclusive_ghost_dofs,
          owned_global_dof_begin,
          owned_global_dof_end);

        element_n_dofs.resize(n_local_elements);
        s(utility::slice(element_n_dofs));

        element_to_rank_maps.resize(n_local_elements);
        for (auto i : utility::range(n_local_elements))
        {
            element_to_rank_maps[i] = new RankDofIndex[element_n_dofs[i]];
        }
        rank_to_global_map.resize(n_owned_dofs + n_local_ghost_dofs + n_exclusive_ghost_dofs);
        s(utility::slice_list(element_to_rank_maps, element_n_dofs),
          utility::slice(rank_to_global_map));
    }
};
}