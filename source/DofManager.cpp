#include "dof_manager/DofManager.h"

#include "dof_manager/DofRanges.h"
#include "dof_manager/detail/EnumerationAlgorithms.h"

#include "utility/MPIUtils.h"

namespace dof_manager
{
    void DofManager::enumerate(
        const MeshType& mesh,
        const ElementScatterMap& scatter_map)
    {
        detail::create_initial_enumeration(mesh, scatter_map, dof_map_);

        enrichment_data_.element_enriched.resize(dof_map_.n_owned_elements);
        std::fill(
            enrichment_data_.element_enriched.begin(), 
            enrichment_data_.element_enriched.end(),
            false);
    }

    void DofManager::set_custom_enumeration(
        const ElementLocalIndex& n_owned_elements,
        const GlobalDofIndex& owned_begin,
        const GlobalDofIndex& owned_end,
        const std::vector<std::vector<RankDofIndex>>& element_to_rank_maps, 
        const std::vector<GlobalDofIndex>& rank_to_global_map)
    {
        dof_map_.rank_to_global_map = rank_to_global_map;
        dof_map_.element_n_dofs.resize(element_to_rank_maps.size(), 0);
        dof_map_.element_to_rank_maps.resize(element_to_rank_maps.size(), nullptr);
        for (auto i : utility::range(element_to_rank_maps.size()))
        {
            const auto n_dofs = static_cast<RankDofIndex>(element_to_rank_maps[i].size());
            dof_map_.element_n_dofs[i] = n_dofs;
            dof_map_.element_to_rank_maps[i] = new RankDofIndex[n_dofs];
            std::copy(
                element_to_rank_maps[i].begin(),
                element_to_rank_maps[i].end(),
                dof_map_.element_to_rank_maps[i]);
        }
        dof_map_.owned_global_dof_begin = owned_begin;
        dof_map_.owned_global_dof_end = owned_end;
        dof_map_.n_owned_dofs = owned_end - owned_begin + 1;
        dof_map_.n_owned_elements = n_owned_elements;
        
        detail::sort_intervals(dof_map_);

        enrichment_data_.element_enriched.resize(dof_map_.n_owned_elements);
        std::fill(
            enrichment_data_.element_enriched.begin(),
            enrichment_data_.element_enriched.end(),
            false);
    }

    RankDofIndex DofManager::add_new_dof(const RankDofIndex original_sibling_dof)
    {
        // Make sure if the sibling dof is also a new dof that we map back
        // to the old rank dof for the sibling
        if (original_sibling_dof >= dof_map_.n_dofs_on_rank())
        {
            const auto old_rank_dof =
                enrichment_data_.new_dof_to_old_rank_map[original_sibling_dof - dof_map_.n_dofs_on_rank()];
            enrichment_data_.new_dof_to_old_rank_map.push_back(old_rank_dof);
        }
        else
        {
            enrichment_data_.new_dof_to_old_rank_map.push_back(original_sibling_dof);
        }

        const RankDofIndex new_rank_number =
            dof_map_.n_dofs_on_rank() + static_cast<RankDofIndex>(enrichment_data_.new_dof_to_old_rank_map.size()) - 1;
        return new_rank_number;
    }

    void DofManager::enrich_element(
        const mesh::LocalIndex& local_element_number, 
        const std::vector<RankDofIndex>& replacement_dofs)
    {
        const auto curr_n_dofs_on_rank =
            dof_map_.n_dofs_on_rank() + enrichment_data_.new_dof_to_old_rank_map.size();
        for (auto& replacement_rank_dof : replacement_dofs)
        {
            _UTIL_ASSERT_C(replacement_rank_dof < curr_n_dofs_on_rank,
                "One of the replacement DoF's was out of range.  Was add_new_dof() called first?");
        }

        delete dof_map_.element_to_rank_maps[local_element_number];
        RankDofIndex* new_dofs = new RankDofIndex[replacement_dofs.size()];
        dof_map_.element_to_rank_maps[local_element_number] = new_dofs;
        std::copy(replacement_dofs.begin(), replacement_dofs.end(), new_dofs);

        dof_map_.element_n_dofs[local_element_number] =
            static_cast<RankDofIndex>(replacement_dofs.size());

        enrichment_data_.element_enriched[local_element_number] = true;
    }

    void DofManager::queue_dof_combination(
        RankDofIndex rank_dof_to_combine,
        RankDofIndex rank_dof_to_combine_into)
    {
        enrichment_data_.combine_dofs(rank_dof_to_combine, rank_dof_to_combine_into);
    }


    RenumberingPhaseData DofManager::renumber(
        const ElementScatterMap& element_scatter)
    {
        auto update_maps = detail::create_maps_for_local_enrichment_and_combinations(
            dof_map_,
            enrichment_data_);

        auto renumbering_data = detail::update_owned_global_numbering(
            update_maps,
            dof_map_);
        
        detail::update_ghost_pre_enrichment_global_numbering(
            element_scatter,
            renumbering_data,
            update_maps,
            dof_map_);

        detail::exchange_new_exclusive_ghost_dofs(
            element_scatter,
            renumbering_data,
            update_maps,
            dof_map_);

        detail::update_element_to_rank_map(update_maps, dof_map_);

        auto global_to_rank_map = detail::get_global_to_rank_map(dof_map_);

        detail::exchange_enriched_ghost_elements(
            enrichment_data_,
            element_scatter,
            global_to_rank_map,
            dof_map_);

        detail::resolve_inter_rank_combinations(
            enrichment_data_,
            global_to_rank_map,
            update_maps,
            renumbering_data,
            dof_map_);

        auto backward_permutation_indices = detail::sort_intervals(dof_map_);

        renumbering_data.permute_owned_renumbering_sources(
            static_cast<std::size_t>(dof_map_.n_owned_dofs),
            backward_permutation_indices,
            detail::get_inverse_map(backward_permutation_indices));

        enrichment_data_.reset();

        return renumbering_data;
    }

    bool DofManager::requires_renumber() const
    {
        return enrichment_data_.dof_to_combine_map.size() > 0 ||
            enrichment_data_.new_dof_to_old_rank_map.size() > 0;
    }
}