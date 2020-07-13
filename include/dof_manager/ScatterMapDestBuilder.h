#pragma once

#include "roaring/roaring.hh"

#include <vector>

namespace dof_manager
{
    /// For a scatter map, each rank will have data to send to a subset of the global set of ranks.
    /// In most situations in our code, each rank can determine from which ranks it requires data,
    /// due to the fact that the mesh includes information about which rank/partition each
    /// node/element belongs to.  However, each rank may not be able to determine which rank to
    /// send data to or what data to send (which element/node that remote rank requires information
    /// about).
    ///
    /// For an element scatter map, each rank assumes one layer of ghost elements, so a rank can
    /// determine what other ranks require its elements as ghosts (and which of its elements appear
    /// as ghosts on the respective remote rank).  This changes if each rank is not guaranteed
    /// to have one layer of ghost elements.
    ///
    /// On the other hand, a node scatter map has no similar guarantees, so each rank must
    /// communicate with the other ranks to figure out what data and to which rank to send.  This
    /// class encapsulates the algorithm to do so, and hopefully allows optimization to make this
    /// as efficient as possible.
    /// 
    /// Usage:
    /// \code{cpp}
    /// ScatterMapDestBuilder dest_builder;
    /// for (auto& n_ptr : mesh.nodes())
    /// {
    ///     if (n_ptr->get_parent_partition() != this_rank)
    ///     {
    ///         dest_builder.add_source_rank(n_ptr->get_parent_partition());
    ///     }
    /// }
    /// auto dest_ranks = dest_builder.get_dest_ranks();
    /// auto source_ranks = dest_builder.get_source_ranks();
    /// \endcode
    class ScatterMapDestBuilder
    {
    public:
        ScatterMapDestBuilder() {}

        /// Adds a source rank for the scatter map.  This can be called multiple times for the
        /// same rank.
        void add_source_rank(int rank);

        /// Returns the unique sorted list of destination ranks that require data from this rank.
        /// Note: should be called only after all neccessary calls to add_source_rank().
        std::vector<int> get_dest_ranks();

        /// Returns the unique, sorted list of source ranks that were added from calls to
        /// add_source_rank().
        /// Note: should be called only after all neccessary calls to add_source_rank().
        std::vector<int> get_source_ranks();

    private:
        Roaring source_ranks_bitset_;
    };
}