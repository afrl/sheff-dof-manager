#include "dof_manager/DofMap.h"

#include "utility/MPIUtils.h"
#include "utility/Range.h"

namespace dof_manager
{
    DofMap::DofMap(const MeshType & mesh)
        : mesh_(mesh),
        n_global_dofs_(0),
        n_all_dofs_on_rank_(0),
        phase_(DofMapPhase::STATIC)
    {
        enumerate_dofs();
    }

    DofMap::~DofMap()
    {
        for (auto& map : element_to_rank_maps_)
        {
            delete[] map;
        }
        for (auto& map : node_to_rank_maps_)
        {
            delete[] map;
        }
    }

    const std::vector<RankDofIndex*>& DofMap::get_element_to_rank_maps() const
    {
        return element_to_rank_maps_;
    }

    const std::vector<ElementDofIndex>& DofMap::get_element_n_dofs() const
    {
        return element_n_dofs_;
    }

    ElementToRankDofMap DofMap::get_element_map(
        const flex_mesh::LocalIndex element_local_number) const
    {
        return ElementToRankDofMap(
            element_to_rank_maps_[element_local_number],
            element_n_dofs_[element_local_number]);
    }

    const std::vector<RankDofIndex*>& DofMap::get_node_to_rank_maps() const
    {
        return node_to_rank_maps_;
    }

    const std::vector<ElementDofIndex>& DofMap::get_node_n_dofs() const
    {
        return node_n_dofs_;
    }

    NodeToRankDofMap DofMap::get_node_map(
        const flex_mesh::LocalIndex node_local_number) const
    {
        return NodeToRankDofMap(
            node_to_rank_maps_[node_local_number],
            node_n_dofs_[node_local_number]);
    }

    void DofMap::start_enrichment_phase()
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::STATIC,
            "This can only be called during a static phase.");

        phase_ = DofMapPhase::ENRICHMENT;
        ep_new_dof_to_old_rank_map_.clear();
    }

    RankDofIndex DofMap::add_new_dof(
        const RankDofIndex original_sibling_dof)
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::ENRICHMENT,
            "This can only be called during an enrichment phase.");

        // Make sure if the sibling dof is also a new dof that we map back
        // to the old rank dof for the sibling
        if (original_sibling_dof >= n_all_dofs_on_rank_)
        {
            const auto old_rank_dof =
                ep_new_dof_to_old_rank_map_[original_sibling_dof - n_all_dofs_on_rank_];
            ep_new_dof_to_old_rank_map_.push_back(old_rank_dof);
        }
        else
        {
            ep_new_dof_to_old_rank_map_.push_back(original_sibling_dof);
        }

        const RankDofIndex new_rank_number =
            n_all_dofs_on_rank_ + static_cast<RankDofIndex>(ep_new_dof_to_old_rank_map_.size()) - 1;
        return new_rank_number;
    }

    void DofMap::enrich_element(
        const flex_mesh::LocalIndex& local_element_number,
        const std::vector<RankDofIndex>& replacement_dofs)
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::ENRICHMENT,
            "This can only be called during an enrichment phase.");

        auto curr_n_dofs_on_rank =
            n_all_dofs_on_rank_ + ep_new_dof_to_old_rank_map_.size();
        for (auto& replacement_rank_dof : replacement_dofs)
        {
            _UTIL_ASSERT_C(replacement_rank_dof < curr_n_dofs_on_rank,
                "One of the replacement DoF's was out of range.  Was add_new_dof() called first?");
        }

        delete element_to_rank_maps_[local_element_number];
        RankDofIndex *new_dofs = new RankDofIndex[replacement_dofs.size()];
        element_to_rank_maps_[local_element_number] = new_dofs;
        std::copy(replacement_dofs.begin(), replacement_dofs.end(), new_dofs);

        element_n_dofs_[local_element_number] =
            static_cast<RankDofIndex>(replacement_dofs.size());
    }

    void DofMap::enrich_node(
        const flex_mesh::LocalIndex& local_node_number,
        RankDofIndex new_rank_dof_start,
        ElementDofIndex n_added_dofs)
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::ENRICHMENT,
            "This can only be called during an enrichment phase.");

        auto curr_n_dofs_on_rank =
            n_all_dofs_on_rank_ + ep_new_dof_to_old_rank_map_.size();
        _UTIL_ASSERT_C(new_rank_dof_start + n_added_dofs <= curr_n_dofs_on_rank,
            "One of the new DoF's was out of range.  Was add_new_dof() called first?");

        const auto n_old_dofs_for_node = node_n_dofs_[local_node_number];
        const auto n_new_dofs_for_node = n_old_dofs_for_node + n_added_dofs;
        auto& node_dof_map = node_to_rank_maps_[local_node_number];

        // Resize the dof map for the node (keeping the older numbering for
        // i = [0, n_old_dofs_for_node)
        RankDofIndex* new_node_dof_map = new RankDofIndex[n_new_dofs_for_node];
        std::copy(node_dof_map, node_dof_map + n_old_dofs_for_node, new_node_dof_map);
        delete[] node_dof_map;
        node_dof_map = new_node_dof_map;
        node_n_dofs_[local_node_number] = n_new_dofs_for_node;

        // Set the new dof numbers
        for (auto i : utility::range_from_zero(n_added_dofs))
        {
            node_dof_map[i + n_old_dofs_for_node] = new_rank_dof_start + i;
        }
    }

    void DofMap::combine_dofs(
        RankDofIndex lhs,
        RankDofIndex rhs)
    {
        _UTIL_ASSERT_C(is_rank_dof_owned(lhs), "Currently, only owned dof's can be combined.");
        _UTIL_ASSERT_C(is_rank_dof_owned(rhs), "Currently, only owned dof's can be combined.");

        const auto lhs_orig = lhs;
        const auto rhs_orig = rhs;

        // Make sure the dof being combined into is not already being combined into another.
        // If that is the case, then combine rank_dof_to_combine into what rank_dof_to_combine_into
        // is aleady being combined into.
        auto find_iter_lhs = ep_dof_to_combine_map_.find(lhs);
        auto find_iter_rhs = ep_dof_to_combine_map_.find(rhs);
        const bool lhs_is_dependent = find_iter_lhs != ep_dof_to_combine_map_.end();
        const bool rhs_is_dependent = find_iter_rhs != ep_dof_to_combine_map_.end();

        // If lhs is already dependent but rhs is not, then swich and substitute
        // existing relation.
        // Example:
        // Current set    Add                     Resulting set
        // A->B           C->D => D->C => D->B    A->B
        // C->B                                   C->B
        // E->D                                   E->B  (from back substitution)
        //                                        D->B  (new relation)
        if (lhs_is_dependent && !rhs_is_dependent)
        {
            // This combination is already taken care of (through the inverse)
            if (rhs == find_iter_lhs->second)
            {
                return;
            }

            std::swap(lhs, rhs);
            rhs = find_iter_lhs->second;
        }
        // If both are dependent then, create a new relation between the new independent
        // dof's (unless the two independent dof's are the same since the new relation is
        // already taken care of).
        // Example:
        // Current set    Add           Resulting set
        // A->B           A->D => B->E  A->E  (from back substitution)
        // C->B                         C->E  (from back substitution)
        // D->E                         D->E
        // F->E                         F->E
        //                              B->E  (new relation)
        else if (lhs_is_dependent && rhs_is_dependent)
        {
            if (find_iter_lhs->second == find_iter_rhs->second)
            {
                return;
            }
            else
            {
                lhs = find_iter_lhs->second;
                rhs = find_iter_rhs->second;
            }
        }
        else if (!lhs_is_dependent && rhs_is_dependent
            && lhs == find_iter_rhs->second)
        {
            return;
        }

        // Back substitution
        // The new relation still needs to be substituted into the existing relations
        // in case lhs appears as an independent dof in any existing relation.
        // Update relation combining something into lhs to now point to rhs.
        auto equals_range = ep_dof_combined_into_map_.equal_range(lhs);
        for (auto iter = equals_range.first; iter != equals_range.second; ++iter)
        {
            auto find_iter = ep_dof_to_combine_map_.find(iter->second);
            if (find_iter->first == rhs)
            {
                ep_dof_to_combine_map_.erase(find_iter);
            }
            else
            {
                find_iter->second = rhs;
            }
        }
        // Nothing is being combined into lhs anymore, so erase it from the map
        ep_dof_combined_into_map_.erase(lhs);

        // Update maps
        ep_dof_to_combine_map_.insert({ lhs, rhs });
        ep_dof_combined_into_map_.insert({ rhs, lhs });
    }

    const std::pair<bool, RankDofIndex> DofMap::is_dof_combined_into_another(
        const RankDofIndex rank_dof_to_combine) const
    {
        auto find_iter = ep_dof_to_combine_map_.find(rank_dof_to_combine);
        if (find_iter == ep_dof_to_combine_map_.end())
        {
            return { false, 0 };
        }
        else
        {
            return { true, find_iter->second };
        }
    }

    const std::vector<RankDofIndex>& DofMap::get_enrichment_new_dof_to_old_rank_map()
    {
        return ep_new_dof_to_old_rank_map_;
    }

    void DofMap::start_renumber_phase()
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::ENRICHMENT,
            "This can only be called during an enrichment phase.");

        renumber_dofs_for_enrichment();
        renumber_dofs_for_combination();
        phase_ = DofMapPhase::RENUMBER;
    }

    const std::vector<GlobalDofIndex>& DofMap::get_renumbering_old_rank_to_global_map() const
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::RENUMBER,
            "This can only be called during a renumbering phase.");

        return rp_old_rank_to_global_map_;
    }

    const std::vector<RankDofIndex>& DofMap::get_renumbering_new_rank_to_old_rank_map() const
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::RENUMBER,
            "This can only be called during a renumbering phase.");

        return rp_new_rank_to_old_rank_map_;
    }

    void DofMap::end_renumber_phase()
    {
        _UTIL_ASSERT_C(phase_ == DofMapPhase::RENUMBER,
            "This can only be called during a renumbering phase.");

        rp_new_rank_to_old_rank_map_.clear();
        rp_old_rank_to_global_map_.clear();
        phase_ = DofMapPhase::STATIC;
    }

    const std::string DofMap::str() const
    {
        std::stringstream ss;
        ss << "Element DoF Map: (" << element_n_dofs_.size() << " elements)" << std::endl;
        for (auto i : utility::range_from_zero(element_n_dofs_.size()))
        {
            for (auto j : utility::range_from_zero(element_n_dofs_[i]))
            {
                ss << " " << element_to_rank_maps_[i][j];
            }
            ss << std::endl;
        }

        ss << "Node DoF Map: (" << node_n_dofs_.size() << " nodes)" << std::endl;
        for (auto i : utility::range_from_zero(node_n_dofs_.size()))
        {
            for (auto j : utility::range_from_zero(node_n_dofs_[i]))
            {
                ss << " " << node_to_rank_maps_[i][j];
            }
            ss << std::endl;
        }


        return ss.str();
    }


    void DofMap::enumerate_dofs()
    {
        auto local_rank = utility::MPIUtils::get_local_rank();

        // For now, assume each node has the same number of DoF's at the start of the analysis.
        // This algorithm will have to be generalized if this is not the case.
        auto initial_dofs_per_node = mesh_.elements().front()->get_n_dofs_per_point();

        element_to_rank_maps_.resize(mesh_.elements().size(), nullptr);
        element_n_dofs_.resize(mesh_.elements().size(), 0);

        node_to_rank_maps_.resize(mesh_.nodes().size(), nullptr);
        node_n_dofs_.resize(mesh_.nodes().size(), initial_dofs_per_node);

        // Count the number of DoF's on this rank in each interval
        n_owned_dofs_ = 0;
        n_lower_ghost_dofs_ = 0;
        n_higher_ghost_dofs_ = 0;
        for (auto& n_ptr : mesh_.nodes())
        {
            if (n_ptr->get_parent_partition() < utility::MPIUtils::get_local_rank())
            {
                n_lower_ghost_dofs_ += initial_dofs_per_node;
            }
            else if (n_ptr->get_parent_partition() == utility::MPIUtils::get_local_rank())
            {
                n_owned_dofs_ += initial_dofs_per_node;
            }
            else
            {
                n_higher_ghost_dofs_ += initial_dofs_per_node;
            }
        }
        n_all_dofs_on_rank_ = n_lower_ghost_dofs_ + n_owned_dofs_ + n_higher_ghost_dofs_;

        // Create the initial node-to-rank maps and rank-to-global map
        // When creating the rank to global map, it is assumed that the node
        // global numbering follows:
        // 1) All owned node numbers by a rank is a continuous interval of integers
        // 2) The intervals owned by each rank are sorted by rank
        rank_to_global_map_.resize(n_all_dofs_on_rank_, 0);
        RankDofIndex next_pre_ghost_index = 0;
        RankDofIndex next_owned_index = n_lower_ghost_dofs_;
        RankDofIndex next_post_ghost_index = n_lower_ghost_dofs_ + n_owned_dofs_;
        for (auto& n_ptr : mesh_.nodes())
        {
            auto& node_dof_map = node_to_rank_maps_[n_ptr->get_local_number()];
            node_dof_map = new RankDofIndex[initial_dofs_per_node];
            if (n_ptr->get_parent_partition() < utility::MPIUtils::get_local_rank())
            {
                for (auto component : utility::range_from_zero(initial_dofs_per_node))
                {
                    const auto global_number =
                        n_ptr->get_global_number() * initial_dofs_per_node + component;
                    rank_to_global_map_[next_pre_ghost_index] = global_number;
                    node_dof_map[component] = next_pre_ghost_index;
                    next_pre_ghost_index++;
                }
            }
            else if (n_ptr->get_parent_partition() == utility::MPIUtils::get_local_rank())
            {
                for (auto component : utility::range_from_zero(initial_dofs_per_node))
                {
                    const auto global_number =
                        n_ptr->get_global_number() * initial_dofs_per_node + component;
                    rank_to_global_map_[next_owned_index] = global_number;
                    node_dof_map[component] = next_owned_index;
                    next_owned_index++;
                }
            }
            else
            {
                for (auto component : utility::range_from_zero(initial_dofs_per_node))
                {
                    const auto global_number =
                        n_ptr->get_global_number() * initial_dofs_per_node + component;
                    rank_to_global_map_[next_post_ghost_index] = global_number;
                    node_dof_map[component] = next_post_ghost_index;
                    next_post_ghost_index++;
                }
            }
        }

        // Create the initial element-rank-map
        n_owned_elements_ = 0;
        const auto this_rank = utility::MPIUtils::get_local_rank();
        for (auto& e_ptr : mesh_.elements())
        {
            const ElementDofIndex n_dofs_for_element =
                static_cast<ElementDofIndex>(e_ptr->get_num_nodes() * initial_dofs_per_node);
            element_to_rank_maps_[e_ptr->get_local_number()] =
                new RankDofIndex[n_dofs_for_element];
            element_n_dofs_[e_ptr->get_local_number()] = n_dofs_for_element;

            auto& element_dof_map = element_to_rank_maps_[e_ptr->get_local_number()];
            ElementDofIndex node_index = 0;
            for (auto& n_ptr : *e_ptr)
            {
                for (auto component : utility::range_from_zero(initial_dofs_per_node))
                {
                    const auto rank_number =
                        node_to_rank_maps_[n_ptr->get_local_number()][component];
                    element_dof_map[node_index * initial_dofs_per_node + component] =
                        rank_number;
                }
                node_index++;
            }

            if (e_ptr->get_parent_partition() == this_rank)
            {
                n_owned_elements_++;
            }
        }

        auto n_owned_dofs_by_rank = utility::MPIUtils::all_gather_vectors<GlobalDofIndex>(
            { static_cast<GlobalDofIndex>(n_owned_dofs_) });

        // Determine the range of dof's this rank owns
        // We assume that each rank will own a contiguous range, so we can determined
        // the owned range of this rank based on the n_owned_dofs in each rank.
        owned_global_dof_begin_ = 0;
        for (auto rank : utility::range_from_zero(utility::MPIUtils::get_local_rank()))
        {
            owned_global_dof_begin_ += n_owned_dofs_by_rank[rank];
        }
        owned_global_dof_end_ = owned_global_dof_begin_ + n_owned_dofs_ - 1;

        n_global_dofs_ = 0;
        for (auto n_owned_dofs_for_rank : n_owned_dofs_by_rank)
        {
            n_global_dofs_ += n_owned_dofs_for_rank;
        }
    }

    void DofMap::renumber_dofs_for_enrichment()
    {
        // Algorithm Description
        //
        // Notes:
        // - At this point element-to-rank and node-to-rank maps do contain all of the
        //   new DoF's since they were modified during the enrichment phase.  However,
        //   the rank numbering will change, invalidating the numbering during this
        //   algorithm until they are updated again.
        // - The rank-to-global map is not updated at all during enrichment phase.
        // - Enrichment phase introduces DoF's to the end of the rank numbering, which
        //   will need to be changed during this algorithm.
        // - For now assume there are no new ghosts, but this will need to be updated if cracks 
        //   are allowed to grow up to partition boundaries.  To alleviate this restriction,
        //   scatter any new DoF's to ranks that have the original sibling dof as a ghost.
        //
        // Steps:
        // 1) Communicate how many new DoF's have been added to each rank
        // 2) Communicate how many DoF's each rank used to own before enrichment began
        // 3) Create a new-to-old map for the rank numbering that also reorganizes the
        //    rank numbering to maintain the sequence:
        //    - ghosts from lower ranks
        //    - owned by this rank
        //    - ghosts from higher ranks
        // 4) Update ep_dof_to_combine_map_ so a later call to renumber_dofs_for_combination()
        //    has an updated map to work with.
        // 4) Create a new rank-to-global map
        // 5) Update the element-to-rank and node-to-rank maps since the DoF's have been
        //    shuffled (but note the element-to-rank and node-to-rank maps were modified
        //    during the enrichment phase)

        // Cache some old counts
        const auto old_n_all_dofs_on_rank = n_all_dofs_on_rank_;
        const auto old_n_owned_and_pre_ghost_dofs =
            n_lower_ghost_dofs_ + n_owned_dofs_;
        const auto old_n_owned_dofs = n_owned_dofs_;

        // Step 1 - as referred to in algorithm outline
        // Find how many new DoF's were created on each rank
        const RankDofIndex n_new_dofs_for_this_rank =
            static_cast<RankDofIndex>(ep_new_dof_to_old_rank_map_.size());
        auto new_dofs_by_rank =
            utility::MPIUtils::all_gather_vectors<RankDofIndex>({ n_new_dofs_for_this_rank });
        RankDofIndex n_global_new_dofs = std::accumulate(new_dofs_by_rank.begin(), new_dofs_by_rank.end(), 0);
        RankDofIndex n_new_dofs_from_lower_ranks = std::accumulate(
            new_dofs_by_rank.begin(),
            new_dofs_by_rank.begin() + utility::MPIUtils::get_local_rank(),
            0);
        std::vector<RankDofIndex> n_new_dofs_from_lower_ranks_by_rank;
        {
            RankDofIndex cumulative_new_dof_count = 0;
            for (auto n_new_dofs_for_rank : new_dofs_by_rank)
            {
                n_new_dofs_from_lower_ranks_by_rank.push_back(cumulative_new_dof_count);
                cumulative_new_dof_count += n_new_dofs_for_rank;
            }
        }

        // Step 2 - as referred to in algorithm outline
        // Find out how many DoF's each rank used to own and the beginning of each range
        // This is used to assess which rank owns a given global dof
        auto old_n_owned_dofs_by_rank =
            utility::MPIUtils::all_gather_vectors<RankDofIndex>({ n_owned_dofs_ });
        std::vector<GlobalDofIndex> old_owned_global_dof_begin_by_rank;
        {
            GlobalDofIndex cumulative_dof_count = 0;
            for (auto n_owned_dofs_for_rank : old_n_owned_dofs_by_rank)
            {
                old_owned_global_dof_begin_by_rank.push_back(cumulative_dof_count);
                cumulative_dof_count += n_owned_dofs_for_rank;
            }
        }

        // Update all of the counts
        n_global_dofs_ += n_global_new_dofs;
        n_all_dofs_on_rank_ += n_new_dofs_for_this_rank;
        n_owned_dofs_ += n_new_dofs_for_this_rank;
        owned_global_dof_begin_ += n_new_dofs_from_lower_ranks;
        owned_global_dof_end_ += n_new_dofs_from_lower_ranks + n_new_dofs_for_this_rank;
        // n_higher_ghost_dofs_ is unchanged since we assume no new ghosts
        // n_lower_ghost_dofs_ is unchanged since we assume no new ghosts

        // Step 3 - as referred to in algorithm outline
        // Update ep_dof_to_combine_map_
        std::map<RankDofIndex, RankDofIndex> updated_ep_dof_to_combine_map;
        for (auto& entry : ep_dof_to_combine_map_)
        {
            std::pair<RankDofIndex, RankDofIndex> updated_entry = entry;
            if (updated_entry.first > old_n_owned_and_pre_ghost_dofs)
            {
                // A new dof index, which needs to be moved to the end of the
                // owned interval (before post-ghosts).
                if (updated_entry.first > old_n_all_dofs_on_rank)
                {
                    updated_entry.first -= n_higher_ghost_dofs_;
                }
                // A post-ghost index, which needs to be moved to the end (due to
                // new dof's on this rank)
                else if (updated_entry.first > old_n_owned_and_pre_ghost_dofs)
                {
                    updated_entry.first += n_new_dofs_for_this_rank;
                }
            }
            if (updated_entry.second > old_n_owned_and_pre_ghost_dofs)
            {
                // A new dof index, which needs to be moved to the end of the
                // owned interval (before post-ghosts).
                if (updated_entry.second > old_n_all_dofs_on_rank)
                {
                    updated_entry.second -= n_higher_ghost_dofs_;
                }
                // A post-ghost index, which needs to be moved to the end (due to
                // new dof's on this rank)
                else if (updated_entry.second > old_n_owned_and_pre_ghost_dofs)
                {
                    updated_entry.second += n_new_dofs_for_this_rank;
                }
            }
            updated_ep_dof_to_combine_map.insert(updated_entry);
        }
        ep_dof_to_combine_map_.swap(updated_ep_dof_to_combine_map);
        updated_ep_dof_to_combine_map.clear();

        // Step 4 - as referred to in algorithm outline
        // Create the new rank to old rank map
        rp_new_rank_to_old_rank_map_.resize(n_all_dofs_on_rank_);
        // Set intervals:
        //  [ pre-ghosts ]
        //  [  old owned ]
        for (auto new_rank_index : utility::range_from_zero(
            n_lower_ghost_dofs_ + old_n_owned_dofs))
        {
            // WARNING: Assumes n_lower_ghost_dofs_ is unchanged
            rp_new_rank_to_old_rank_map_[new_rank_index] = new_rank_index;
        }
        // Insert the new DoF's at the end of the owned range
        // Set the interval: [ new owned ]
        for (auto new_dof_index : utility::range_from_zero(n_new_dofs_for_this_rank))
        {
            // WARNING: Assumes n_lower_ghost_dofs_ is unchanged
            auto const new_rank_index =
                n_lower_ghost_dofs_ + old_n_owned_dofs + new_dof_index;
            rp_new_rank_to_old_rank_map_[new_rank_index] =
                ep_new_dof_to_old_rank_map_[new_dof_index];
        }
        // Set the interval: [ post-ghosts ]
        for (auto new_rank_index : utility::range(
            n_lower_ghost_dofs_ + n_owned_dofs_ + n_new_dofs_for_this_rank,
            n_lower_ghost_dofs_ + n_owned_dofs_ + n_new_dofs_for_this_rank + n_higher_ghost_dofs_))
        {
            // WARNING: Assumes n_lower_ghost_dofs_ and n_higher_ghost_dofs_ is unchanged
            rp_new_rank_to_old_rank_map_[new_rank_index] = new_rank_index - n_new_dofs_for_this_rank;
        }
        ep_new_dof_to_old_rank_map_.clear();

        // Create the new rank-to-global map
        rp_old_rank_to_global_map_.swap(rank_to_global_map_);
        rank_to_global_map_.resize(n_all_dofs_on_rank_);
        // Pre-ghost interval
        // WARNING: Assumes n_lower_ghost_dofs_ is unchanged
        for (auto i : utility::range_from_zero(n_lower_ghost_dofs_))
        {
            const auto old_global_index = rp_old_rank_to_global_map_[i];
            // Find which rank the old global index belonged to
            for (auto rank : utility::range_from_zero(utility::MPIUtils::get_local_rank()))
            {
                // Due to the sequential ordering, the first rank that index is
                // greater than is the rank that owns it.
                if (old_global_index > old_owned_global_dof_begin_by_rank[rank])
                {
                    rank_to_global_map_[i] =
                        old_global_index + n_new_dofs_from_lower_ranks_by_rank[rank];
                    break;
                }
            }
        }
        // Owned interval before enrichment
        for (auto i : utility::range(n_lower_ghost_dofs_,
            n_lower_ghost_dofs_ + old_n_owned_dofs))
        {
            const auto old_rank_index = rp_new_rank_to_old_rank_map_[i];
            const auto old_global_index = rp_old_rank_to_global_map_[old_rank_index];
            rank_to_global_map_[i] =
                old_global_index + n_new_dofs_from_lower_ranks;
        }
		// New owned DoF's interval
		for (auto i : utility::range_from_zero(n_new_dofs_for_this_rank))
		{
			rank_to_global_map_[i + n_lower_ghost_dofs_ + old_n_owned_dofs] =
				i + rp_old_rank_to_global_map_.back() + n_new_dofs_from_lower_ranks + 1;
		}
        // Post-ghost interval
        // WARNING: Assumes n_lower_ghost_dofs_ and n_higher_ghost_dofs_ is unchanged
        for (auto i : utility::range(n_lower_ghost_dofs_ + n_owned_dofs_,
            n_lower_ghost_dofs_ + n_owned_dofs_ + n_higher_ghost_dofs_))
        {
            const auto old_rank_index = rp_new_rank_to_old_rank_map_[i];
            const auto old_global_index = rp_old_rank_to_global_map_[old_rank_index];
            // Find which rank the old global index belonged to
            for (auto rank : utility::range(
                utility::MPIUtils::get_local_rank() + 1,
                utility::MPIUtils::get_num_local_ranks()))
            {
                // Due to the sequential ordering, the first rank that index is
                // greater than is the rank that owns it.
                if (old_global_index > old_owned_global_dof_begin_by_rank[rank])
                {
                    rank_to_global_map_[i] =
                        old_global_index + n_new_dofs_from_lower_ranks_by_rank[rank];
                    break;
                }
            }
        }

        // Step 5 - as referred to in algorithm outline
        // Finally, update the element-to-rank and node-to-rank maps, since the rank numbering
        // during enrichment was:
        //  [ pre-ghosts  ] ‾‾\
        //  [  old owned  ]    | is of length: old_n_all_dofs_on_rank
        //  [ post-ghosts ] __/ 
        //  [  new owned  ]
        // but we need to change it to be:
        //  [ pre-ghosts  ] ‾‾\
        //  [  old owned  ] __/ is of length: old_n_owned_and_pre_ghost_dofs
        //  [  new owned  ]
        //  [ post-ghosts ]
        // We could generalize the renumbering to take a full rank-to-rank map, but
        // for this particular permutation, it is cheaper to just use offsets.
        for (auto local_element_number : utility::range_from_zero(element_to_rank_maps_.size()))
        {
            auto& element_to_rank_map = element_to_rank_maps_[local_element_number];
            for (auto i : utility::range_from_zero(element_n_dofs_[local_element_number]))
            {
                if (element_to_rank_map[i] > old_n_owned_and_pre_ghost_dofs)
                {
                    // A new dof index, which needs to be moved to the end of the
                    // owned interval (before post-ghosts).
                    if (element_to_rank_map[i] > old_n_all_dofs_on_rank)
                    {
                        element_to_rank_map[i] -= n_higher_ghost_dofs_;
                    }
					// A post-ghost index, which needs to be moved to the end (due to
					// new dof's on this rank)
					else if (element_to_rank_map[i] > old_n_owned_and_pre_ghost_dofs)
					{
						element_to_rank_map[i] += n_new_dofs_for_this_rank;
					}
                }
            }
        }
        for (auto local_node_number : utility::range_from_zero(node_to_rank_maps_.size()))
        {
            auto& node_to_rank_map = node_to_rank_maps_[local_node_number];
            for (auto i : utility::range_from_zero(node_n_dofs_[local_node_number]))
            {
                if (node_to_rank_map[i] > old_n_owned_and_pre_ghost_dofs)
                {
					// A new dof index, which needs to be moved to the end of the
					// owned interval (before post-ghosts).
					if (node_to_rank_map[i] > old_n_all_dofs_on_rank)
					{
						node_to_rank_map[i] -= n_higher_ghost_dofs_;
					}
					// A post-ghost index, which needs to be moved to the end (due to
					// new dof's on this rank)
					else if (node_to_rank_map[i] > old_n_owned_and_pre_ghost_dofs)
					{
						node_to_rank_map[i] += n_new_dofs_for_this_rank;
					}
                }
            }
        }
    }

    void DofMap::renumber_dofs_for_combination()
    {
        // Algorithm Description
        //
        // Notes:
        // - At this point these data structures have a valid numbering:
        //    * element-to-rank maps
        //    * node-to-rank maps
        //    * rank-to-global map
        //    * old rank-to-global map
        //    * new rank to old rank map
        //   That maintains the sequence:
        //    - ghosts from lower ranks
        //    - owned by this rank
        //    - ghosts from higher ranks
        // - Though the numbering is valid, dof's that should be combined have not been
        //   eliminated from the numbering yet.
        // - Both ep_dof_to_combine_map_ and ep_dof_combined_into_map_ refer to dof's in the
        //   new rank numbering, since they were updated during renumber_dofs_for_enrichment().
        // - ep_new_dof_to_old_rank_map_ is empty at this point.
        // - For now, assume no ghosts have been combined.  Even if this restriction is lifted,
        //   it is safe to assume for this step, but a new step will be needed after this is complete
        //   to combined dof's across rank boundaries. See docstring for DofMap::combine_dofs()
        //
        // Steps:
        // 1) Communicate how many DoF's will be eliminated on each rank
        // 2) Communicate how many DoF's each rank owns before elimination.  This is needed
        //    to infer what rank owns a given global dof during step 3.
        // 3) Eliminate dof's and update numbering for:
        //    - new rank-to-global map (stored in rank_to_global_map_)
        //    - new-to-old rank numbering map (stored in rp_new_rank_to_old_rank_map_)
        //    and build a map from rank numbering before elimination to after elimination.
        //    These are combined because they use the exact same loop structure.
        // 4) Eliminate dof's and update numbering for (using map from step 4):
        //    - element-to-rank map
        //    - node-to-rank map

        // Cache some old counts
        const auto old_n_all_dofs_on_rank = n_all_dofs_on_rank_;
        const auto old_n_owned_and_pre_ghost_dofs = n_lower_ghost_dofs_ + n_owned_dofs_;
        const auto old_n_owned_dofs = n_owned_dofs_;

        // Step 1 - as referred to in algorithm outline
        // Find how many DoF's will be eliminated on each rank
        const RankDofIndex n_dofs_to_eliminate_for_this_rank =
            static_cast<RankDofIndex>(ep_dof_to_combine_map_.size());
        auto n_dofs_to_eliminate_by_rank =
            utility::MPIUtils::all_gather_vectors<RankDofIndex>({ n_dofs_to_eliminate_for_this_rank });
        RankDofIndex n_global_dofs_to_eliminate = std::accumulate(
            n_dofs_to_eliminate_by_rank.begin(),
            n_dofs_to_eliminate_by_rank.end(),
            0);
        RankDofIndex n_dofs_to_eliminate_on_lower_ranks = std::accumulate(
            n_dofs_to_eliminate_by_rank.begin(),
            n_dofs_to_eliminate_by_rank.begin() + utility::MPIUtils::get_local_rank(),
            0);
        std::vector<RankDofIndex> n_dofs_to_eliminate_on_lower_ranks_by_rank;
        {
            RankDofIndex cumulative_dof_count = 0;
            for (auto n_dofs_to_eliminate_for_rank : n_dofs_to_eliminate_by_rank)
            {
                n_dofs_to_eliminate_on_lower_ranks_by_rank.push_back(cumulative_dof_count);
                cumulative_dof_count += n_dofs_to_eliminate_for_rank;
            }
        }

        // Step 2 - as referred to in algorithm outline
        // Find out how many DoF's each rank used to own and the beginning of each range
        // This is used to assess which rank owns a given global dof
        auto old_n_owned_dofs_by_rank =
            utility::MPIUtils::all_gather_vectors<RankDofIndex>({ n_owned_dofs_ });
        std::vector<GlobalDofIndex> old_owned_global_dof_begin_by_rank;
        {
            GlobalDofIndex cumulative_dof_count = 0;
            for (auto n_owned_dofs_for_rank : old_n_owned_dofs_by_rank)
            {
                old_owned_global_dof_begin_by_rank.push_back(cumulative_dof_count);
                cumulative_dof_count += n_owned_dofs_for_rank;
            }
        }

        // Update all of the counts
        n_global_dofs_ -= n_global_dofs_to_eliminate;
        n_all_dofs_on_rank_ -= n_dofs_to_eliminate_for_this_rank;
        n_owned_dofs_ -= n_dofs_to_eliminate_for_this_rank;
        owned_global_dof_begin_ -= n_dofs_to_eliminate_on_lower_ranks;
        owned_global_dof_end_ -= n_dofs_to_eliminate_on_lower_ranks + n_dofs_to_eliminate_for_this_rank;
        // n_higher_ghost_dofs_ is unchanged since we assume no new ghosts
        // n_lower_ghost_dofs_ is unchanged since we assume no new ghosts

        // Step 3 - as referred to in algorithm outline
        // Now update the rank-to-global-map and rp_new_rank_to_old_rank_map_
        std::vector<GlobalDofIndex> updated_rank_to_global_map;
        updated_rank_to_global_map.reserve(n_all_dofs_on_rank_);
        std::vector<GlobalDofIndex> updated_rp_new_rank_to_old_rank_map;
        updated_rp_new_rank_to_old_rank_map.reserve(n_all_dofs_on_rank_);
        // Also build a map from rank numbering before elimination to after elimination
        std::vector<RankDofIndex> before_to_after_elimination_map(old_n_all_dofs_on_rank);
        // We know that ep_dof_to_combine_map_ is sorted, so we can take advantage of
        // this to avoid a bunch of searches.
        auto curr_dof_to_combine_iter = ep_dof_to_combine_map_.begin();
        auto cumulative_dofs_eliminated_on_this_rank = 0;
        for (auto index_before_elimination : utility::range_from_zero(old_n_all_dofs_on_rank))
        {
            // This dof remains, but the number needs to be shifted to accomodate
            // dof eliminated earlier in the numbering.
            if (curr_dof_to_combine_iter == ep_dof_to_combine_map_.end()
                || index_before_elimination != curr_dof_to_combine_iter->first)
            {
                // WARNING: Assumes n_lower_ghost_dofs_ is unchanged
                before_to_after_elimination_map[index_before_elimination] =
                    index_before_elimination -
                    cumulative_dofs_eliminated_on_this_rank;
                updated_rank_to_global_map.push_back(
                    rank_to_global_map_[index_before_elimination] - cumulative_dofs_eliminated_on_this_rank);
                updated_rp_new_rank_to_old_rank_map.push_back(
                    rp_new_rank_to_old_rank_map_[index_before_elimination]);
            }
            // This dof is eliminated, skip it but update counters
            else
            {
                curr_dof_to_combine_iter++;
                cumulative_dofs_eliminated_on_this_rank++;
            }
        }
        rank_to_global_map_.swap(updated_rank_to_global_map);
        updated_rank_to_global_map.clear();
        rp_new_rank_to_old_rank_map_.swap(updated_rp_new_rank_to_old_rank_map);
        updated_rp_new_rank_to_old_rank_map.clear();

        // During the first pass, before_to_after_elimination_map has been populated
        // with the maps to all dof's that will stay.  However, a second pass is needed
        // to update maps to dof's that been eliminated.  They could not be set during
        // the first pass since ep_dof_to_combine_map_ points to the old number of dof's
        // that will remain, and though the keys are sorted, the values are not, so no
        // implicit relation exists.
        curr_dof_to_combine_iter = ep_dof_to_combine_map_.begin();
        for (auto index_before_elimination : utility::range_from_zero(old_n_all_dofs_on_rank))
        {
            if (curr_dof_to_combine_iter != ep_dof_to_combine_map_.end())
            {
                if (index_before_elimination == curr_dof_to_combine_iter->first)
                {
                    before_to_after_elimination_map[index_before_elimination] =
                        before_to_after_elimination_map[curr_dof_to_combine_iter->second];
                    curr_dof_to_combine_iter++;
                }
            }
        }

        // Step 4 - as referred to in algorithm outline
        // Update the element-to-rank maps
        for (auto e_i : utility::indices(element_to_rank_maps_))
        {
            auto& element_to_rank_map = element_to_rank_maps_[e_i];
            for (auto i : utility::range_from_zero(element_n_dofs_[e_i]))
            {
                element_to_rank_map[i] =
                    before_to_after_elimination_map[element_to_rank_map[i]];
            }
        }
        // Update the node-to-rank maps
        for (auto n_i : utility::indices(node_to_rank_maps_))
        {
            auto& node_to_rank_map = node_to_rank_maps_[n_i];
            for (auto i : utility::range_from_zero(node_n_dofs_[n_i]))
            {
                node_to_rank_map[i] =
                    before_to_after_elimination_map[node_to_rank_map[i]];
            }
        }

        // Debug asserts
        for (auto& after_dof : before_to_after_elimination_map)
        {
            assert(after_dof >= 0
                && after_dof < n_all_dofs_on_rank_);
        }
        for (auto e_i : utility::indices(element_to_rank_maps_))
        {
            auto& element_to_rank_map = element_to_rank_maps_[e_i];
            for (auto i : utility::range_from_zero(element_n_dofs_[e_i]))
            {
                assert(element_to_rank_map[i] >= 0
                    && element_to_rank_map[i] < n_all_dofs_on_rank_);
            }
        }
        for (auto n_i : utility::indices(node_to_rank_maps_))
        {
            auto& node_to_rank_map = node_to_rank_maps_[n_i];
            for (auto i : utility::range_from_zero(node_n_dofs_[n_i]))
            {
                assert(node_to_rank_map[i] >= 0
                    && node_to_rank_map[i] < n_all_dofs_on_rank_);
            }
        }

        before_to_after_elimination_map.clear();

        ep_dof_to_combine_map_.clear();
        ep_dof_combined_into_map_.clear();
    }

}
