#pragma once

#include "SingleDofMap.h"

#include "flex_mesh/mesh/Mesh.h"
#include "flex_mesh/node/Node.h"
#include "finite_element/FiniteElement.h"

#include <map>
#include <vector>

namespace dof_manager
{
    enum class DofMapPhase : char
    {
        STATIC, ///< All maps are static, allowing global linear algebra operations
        ENRICHMENT, ///< element-to-rank maps are modified as new DoF's are added (linear algebra operations not allowed)
        RENUMBER ///< element-to-rank maps and rank-to-global maps are updated (linear algebra operations not allowed)
    };

    /// There are three types of numbering:
    /// 1) Element-level numbering, which is 0-based and goes to the # of DoF's for the
    ///    element.
    ///
    /// 2) Rank-level numbering, which is 0-based and goes to the # of DoF's on the rank,
    ///    including the ghost and owned DoF's.  The structure of this ordering is:
    ///     [ ghost dof's owned by lower ranks  ]  ‾\ size of this range is = n_lower_ghost_dofs_
    ///     [                ...                ]  _/
    ///     [           owned dof's             ]  ‾\ size of this range is = n_owned_dofs_
    ///     [                ...                ]  _/
    ///     [ ghost dof's owned by higher ranks ]  ‾\ size of this range is = n_upper_ghost_dofs_
    ///     [                ...                ]  _/
    ///    
    /// 3) Global-level numbering, which is 0-based and goes to the # of DoF's for the
    ///    entire domain (possibly beyond what this rank knows about) and must be unique
    ///    across the domain.  The structure of this numbering scheme is very similar to
    ///    that of rank-level numbering.  However, it should be noted that a one-to-one
    ///    map between the relative order within each of the three blocks is not assumed 
    ///    for the rank and global numbering.  In other words, the first owned DoF in the
    ///    rank numbering is not neccessarily the first DoF in the global numbering.
    ///
    /// To allow a finite element code to assemble element-level matrices and vectors to
    /// global representations, maps from element-to-rank and rank-to-global are stored by
    /// this class.
    ///
    /// For XFEM analyses, new DoF's are added, creating some complexity.  To accommodate
    /// this complexity, three phases are introduced:
    /// 1) Static phase - the maps are static and no new DoF's can be inserted.  During
    ///    this phase, assembly and global linear operations can take place.
    ///
    /// 2) Enrichment phase - the element-to-rank map is updated during this phase as new
    ///    DoF's are added to the system, but the rank-to-global map is invalid during this
    ///    phase.  No global linear algebra operations are allowed.  It is important to note
    ///    that during this phase, the linear algebra containers will not be updated after
    ///    new DoF's are added (the resize happens later).  However, in the sparse_linear_algebra
    ///    module, Vector::get_element_part will know this.  If an element DoF map has a rank
    ///    number that is greater than the # of DoF's known to the Vector, then it will use
    ///    a second map maintained by this class to access which old rank-based number the
    ///    value of the new DoF should be pulled from.  This will allow the code to calculate
    ///    element-level quantities needed during the enrichment phase.
    ///    
    ///    Additionally, it should be noted that the structure of the rank-based numbering
    ///    described earlier will be violated during this phase.  However, the expected
    ///    structure will be restored in the renumber phase (next).
    ///
    /// 3) Renumber phase - all maps are updated with new numbering in order to restore the
    ///    (pre-ghost ..., owned ..., post-ghost ...) structure.  During this phase, a map
    ///    is maintained from the new rank-numbering to the old-rank numbering in order to
    ///    support the linear algebra containers reusing the existing values.  Otherwise,
    ///    a new assembly step would be required.
    ///
    /// TODO A scatter map needs to be implemented in this class to know which owned dof's
    /// appear as ghosts in other ranks (and which ranks).  Using the map, the logic can
    /// be added to inform other ranks about calls to add_new_dof(original_sibling_dof) if
    /// original_sibling_dof appears as a ghost for the respective rank.  The renumbering
    /// step would need to be updated to account for the number of ghosts changing, and the
    /// enrichment step would need to be modified to accomdate a ghost update.
    /// Also, when the scatter map is implemented, the linear algebra containers might just
    /// want to reuse it.
    class DofMap
    {
    public:
        using MeshType = flex_mesh::Mesh<flex_mesh::Node, finite_element::FiniteElement>;

        /// The constructor will enumerate DoF's to the elements in the mesh provided.
        DofMap(const MeshType & mesh);
        ~DofMap();

		///////////////
		// Accessors //
		///////////////

        // Note: should we just return the vector&'s?  This would avoid invalid pointers
        // for the vectors during the enrichment phase if new elements are added.

        /// Access the array of element-based to rank-based DoF number maps.
        /// The first index is the local element number and the second index is element-level
        /// DoF number rank-based.  The first axis has length n_all_dofs_on_rank_.  The second
        /// has length element_n_dofs_[local_element_number].
        const std::vector<RankDofIndex *>& get_element_to_rank_maps() const;
        /// Access the array of the number of DoF's each element.
        const std::vector<ElementDofIndex>& get_element_n_dofs() const;
        /// Returns an object that wraps a single element's DoF map to provide convenient
        /// access iterators for loops.
        ElementToRankDofMap get_element_map(
            const flex_mesh::LocalIndex element_local_number) const;

		/// Access the node to rank DoF map.  The first index is the local node number
		/// and the second index is the rank-based DoF number of the node-level DoF index
		/// (ordered by enrichment then by field component).
        const std::vector<RankDofIndex*>& get_node_to_rank_maps() const;
		/// Access the number of DoF's each node.
        const std::vector<ElementDofIndex>& get_node_n_dofs() const;
        /// Returns an object that wraps a single nodes's DoF map to provide convenient
        /// access iterators for loops.
        NodeToRankDofMap get_node_map(
            const flex_mesh::LocalIndex node_local_number) const;

        inline const GlobalDofIndex& get_n_global_dofs() const
        {
            return n_global_dofs_;
        }
        inline const RankDofIndex& get_n_owned_dofs() const
        {
            return n_owned_dofs_;
        }
        inline const RankDofIndex& get_n_ghost_dofs_from_lower_ranks() const
        {
            return n_lower_ghost_dofs_;
        }
        inline const RankDofIndex& get_n_ghost_dofs_from_higher_ranks() const
        {
            return n_higher_ghost_dofs_;
        }
        inline const GlobalDofIndex& get_owned_global_dof_begin() const
        {
            return owned_global_dof_begin_;
        }
        inline const GlobalDofIndex& get_owned_global_dof_end() const
        {
            return owned_global_dof_end_;
        }
        inline const flex_mesh::LocalIndex& get_n_owned_elements() const
        {
            return n_owned_elements_;
        }
        /// Returns the rank to global map.  The length of the vector will be the number
        /// of all DoF's on this rank.
        inline const std::vector<GlobalDofIndex>& get_rank_to_global_map() const
        {
            return rank_to_global_map_;
        }

        //////////////////////////////
        // Enrichment Phase Methods //
        //////////////////////////////

        /// Starts an enrichment phase, allowing calls to enrich_element and
        /// enrich_node.  During this phase, new DoF's can be added to the system
        /// and will be tracked.
        /// The element to rank maps and node to rank maps will be updated during
        /// this phase as enrich_element is called.
        void start_enrichment_phase();

        /// Adds a new dof to the system adn returns the rank-level number, which
        /// can be used in the element-to-rank and node-to-element maps.
        ///
        /// \param original_sibling_dof - the existing dof that new_dof was created
        ///     to be a sibling to.  In the xfem code, when an element is enriched,
        ///     the old dof is used for one basal field, while a new dof is created
        ///     for the other basal field.  This parameter is the old dof.
        ///
        /// Note: because of this modifies counters and containers, enrichment of
        ///       elements MUST be sequential on each rank or a data race will ensue.
        ///       However, atomic operations could be implemented in the future.
        /// Note: if new DoF's need to be added for other reasons than XFEM, then
        ///       another version of add_new_dof should be created.
        const RankDofIndex add_new_dof(
            const RankDofIndex original_sibling_dof);

        /// Assign a new set of DoF's to an existing element, enriching the number DoF's
        /// for that element.  Any new DoF's will be owned by the rank that enriched the
        /// element.
        void enrich_element(
            const flex_mesh::LocalIndex& local_element_number,
            const std::vector<RankDofIndex>& replacement_dofs);

        /// Adds (new_global_dof_start, new_global_dof_start + 1, ...) up to n_new_dofs
        /// to the end of the node dof map for boundary conditions.  Note: all of the
        /// components for a field should be added at once, since the boundary
        /// conditions assume the ordering.
        void enrich_node(
            const flex_mesh::LocalIndex& local_node_number,
            RankDofIndex new_rank_dof_start,
            ElementDofIndex n_new_dofs);

        /// Mark that a dof should be combined into another during the next renumbering phase.
        /// This should be relatively rare, but sometimes, a compatibility of two enrichments
        /// will require combining dof's.  During the enrichment phase, either dof can be used
        /// interchangably, and during renumbering, one will be combined into another.
        ///
        /// Restriction: For now, rank_dof_to_combine and rank_dof_to_combine_into must both
        ///              be owned dof's. MPI communication will be needed for this to be relaxed.
        ///
        /// Proposal for combining dof's owned by different ranks.  Accumulate the combinations
        /// into a vector<pair<RankDofIndex, RankDofIndex>> if rank_dof_to_combine and/or
        /// rank_dof_to_combine_into is not owned.  After renumbering inserts new dof's then
        /// eliminates owned dof's from combinations, add a new step where dof's are combined across
        /// ranks.  Each rank will have to know about all combinations across rank boundaries to
        /// ensure they can keep their maps up to date, but at least only the combinations across
        /// ranks will be needed by all ranks (instead of all combinations internal to each rank).
        /// Unfortunately, a global-to-rank map might be needed since a rank doens't know about
        /// another rank's numbering.  But the global-to-rank map would only be needed in this
        /// step and only really needed for ghosts since there is an implicit global-to-rank map
        /// for owned dof's.
        void combine_dofs(
            RankDofIndex rank_dof_to_combine,
            RankDofIndex rank_dof_to_combine_into);

        /// This is mostly used for unit-testing
        const std::pair<bool, RankDofIndex> is_dof_combined_into_another(
            const RankDofIndex rank_dof_to_combine) const;

        // Something like this may eventually be implemented
        //LocalIndex add_element(const std::vector<GlobalIndex>& element_global_dofs);

        /// For each new DoF added during an enrichment phase, the vector returned by
        /// reference will contain the corresponding old rank-level number (before
        /// the enrichments occured).  This should be used by calls to
        /// sparse_linear_algebra::Vector::get_element_part() to get an updated
        /// element part of the global vector without actually updating the global
        /// vector.  A reference to this vector can be kept indefinitely by the
        /// sparse_linear_algebra::Vector class, though the vector will be empty
        /// outside of the enrichment phase.
        ///
        /// Note: this map does not know anything about combined dof's.  Combined
        /// dof's will be incorporated into the numbering during the renumbering phase.
        const std::vector<RankDofIndex>& get_enrichment_new_dof_to_old_rank_map();

        ////////////////////////////
        // Renumber Phase Methods //
        ////////////////////////////

        /// Completes an enrichment phase and starts the renumber phase.
        /// Calls to enrich_element and enrich_node will no longer be allowed.
        /// All maps are updated to enforce the assumed structure. \see DofMap
        void start_renumber_phase();

        /// After start_renumber_phase() has been called, the renumbering is complete
        /// within the DofMap object, but the global linear algebra containers have not
        /// been updated.  This returns a map of the old rank numbering (before the last
        /// enrichment phase) to the old global numbering.  The map is just cached before
        /// renumbering occurs, as it is needed for the linear algebra containers.
        /// A reference to this can be held as long as the DofMap object lives, but the
        /// vectors will be empty after end_renumber_phase is called.
        const std::vector<GlobalDofIndex>& get_renumbering_old_rank_to_global_map() const;
        const std::vector<RankDofIndex>& get_renumbering_new_rank_to_old_rank_map() const;

        /// This should be called after the linear algebra containers have been permuted 
        /// to accomodate the new DoF's.  This will clean up the structures and set the
        /// phase to STATIC, which allows linear algebra operations.
        void end_renumber_phase();

        ///////////
        // Misc. //
        ///////////

        const std::string str() const;

    private:

        using DofIndexDelta = int64_t;

        ///////////////////////
        // Common properties //
        ///////////////////////

        const MeshType & mesh_;

        flex_mesh::LocalIndex n_owned_elements_;
        std::vector<RankDofIndex*> element_to_rank_maps_;
        std::vector<ElementDofIndex> element_n_dofs_;
		std::vector<RankDofIndex*> node_to_rank_maps_;
		std::vector<ElementDofIndex> node_n_dofs_;

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
        std::vector<GlobalDofIndex> rank_to_global_map_;

        /// The total number of DoF's in the global system
        /// Note: during the enrichment phase, this will be incorrect as new DoF's
        //        do not have global numbers until the renumbering phase.
        GlobalDofIndex n_global_dofs_;
        /// The number of all DoF's on this rank (includes owned and ghosted)
        /// Note: during the enrichment phase, this will actually be number of all
        ///       DoF's as there were before the enrichment phase began.  It does
        ///       not take into account any added DoF's.
        RankDofIndex n_all_dofs_on_rank_;
        /// The number of DoF's owned by this rank (excludes ghosts).
        /// Due to the global number scheme, this is equivalent to:
        /// owned_global_dof_end_ - owned_global_dof_begin_
        /// Note: during the enrichment phase, this will be incorrect as new DoF's
        //        do not have global numbers until the renumbering phase.
        RankDofIndex n_owned_dofs_;
        /// Since global numbering scheme will maintain that the DoF's owned by this rank
        /// lie within a continuous interval (with each rank's interval sorted by rank),
        /// this is the beginning of the range (inclusive).
        /// Note: during the enrichment phase, this will be incorrect as new DoF's
        //        do not have global numbers until the renumbering phase.
        GlobalDofIndex owned_global_dof_begin_;
        /// The end of the global owned range (inclusive). \see owned_global_dof_begin_
        /// Note: during the enrichment phase, this will be incorrect as new DoF's
        //        do not have global numbers until the renumbering phase.
        GlobalDofIndex owned_global_dof_end_;
        /// The number of DoF's need as ghosts on this rank that are owned by lower ranks
        /// Note: during the enrichment phase, this will be incorrect, but it will be
        ///       updated during the renumbering phase.
        RankDofIndex n_lower_ghost_dofs_;
        /// The number of DoF's need as ghosts on this rank that are owned by higher ranks
        /// Note: during the enrichment phase, this will be incorrect, but it will be
        ///       updated during the renumbering phase.
        RankDofIndex n_higher_ghost_dofs_;
        /// Tracks what phase the DofMap object is in, allowing some basic assertions to
        /// avoid methods being called out of phase.
        DofMapPhase phase_;

        /////////////////////////////////
        // Enrichment phase properties //
        /////////////////////////////////

        /// \see DofMap::get_new_dof_to_old_rank_map
        std::vector<RankDofIndex> ep_new_dof_to_old_rank_map_;
        /// \see DofMap::combine_dofs
        /// A dof to be combined into another is a 1-to-1 relationship.
        std::map<RankDofIndex, RankDofIndex> ep_dof_to_combine_map_;
        /// The opposite of ep_dof_to_combine_map_, allowing looking up if a given dof is being
        /// combined into.  The key of this structure is the dof being combined into, and the value
        /// is a dof being combined.  There can be multiple dof's combined into a single dof, so
        /// a multimap is used.
        std::multimap<RankDofIndex, RankDofIndex> ep_dof_combined_into_map_;

        //////////////////////////////////
        // Renumbering phase properties //
        //////////////////////////////////

        std::vector<GlobalDofIndex> rp_old_rank_to_global_map_;
        std::vector<GlobalDofIndex> rp_new_rank_to_old_rank_map_;

        ////////////////////
        // Helper Methods //
        ////////////////////

        void enumerate_dofs();
        /// Renumbers dof's to accommodate new dof's that were added via calls
        /// to add_new_dof()
        void renumber_dofs_for_enrichment();
        /// Renumbers dof's to remove dof's that were denoted to be combined via
        /// a call to combine_dofs()
        void renumber_dofs_for_combination();

        inline const bool is_rank_dof_owned(
            const RankDofIndex& dof) const
        {
            return (dof >= n_lower_ghost_dofs_ && dof < n_lower_ghost_dofs_ + n_owned_dofs_)
                || dof >= n_all_dofs_on_rank_;
        }

        inline const bool is_global_dof_owned(
            const GlobalDofIndex& dof) const
        {
            return dof >= owned_global_dof_begin_ && dof < owned_global_dof_end_;
        }
    };
}