#include "dof_manager/detail/EnumerationAlgorithms.h"
#include "dof_manager/DofManager.h"
#include "utility/MPIUtils.h"

#include <thread>
#include <chrono> 

using namespace dof_manager;

void assert_element_to_rank(
    const DofMap& dof_map,
    const std::vector<std::vector<RankDofIndex>>& element_to_rank)
{
    assert(dof_map.element_to_rank_maps.size() == element_to_rank.size());
    for (auto i : utility::range(element_to_rank.size()))
    {
        assert(element_to_rank[i].size() == dof_map.element_n_dofs[i]);
        for (auto j : utility::range(element_to_rank[i].size()))
        {
            assert(element_to_rank[i][j] == dof_map.element_to_rank_maps[i][j]);
        }
    }
}

int main(int argc, char** argv)
{
    utility::MPIUtils::init(&argc, &argv);

    std::this_thread::sleep_for(std::chrono::seconds(20));

    // Create a mesh that looks like:
    // 3     4     7     8     9
    // o-----o-----o-----o-----o
    // |  0  |  1  |  2  |  3  |
    // o-----o-----o-----o-----o
    // 0     1     2     5     6
    finite_element::FiniteElementMesh fe_mesh;
    if (utility::MPIUtils::get_local_rank() == 0)
    {
        mesh::Mesh mesh;

        mesh.nodes().add({ 0, 0, 0 }, 0, 0);
        mesh.nodes().add({ 1, 0, 0 }, 1, 0);
        mesh.nodes().add({ 2, 0, 0 }, 2, 0);
        mesh.nodes().add({ 0, 1, 0 }, 3, 0);
        mesh.nodes().add({ 1, 1, 0 }, 4, 0);
        mesh.nodes().add({ 2, 1, 0 }, 7, 1);
        mesh.nodes().add({ 3, 0, 0 }, 5, 1);
        mesh.nodes().add({ 3, 1, 0 }, 8, 1);

        mesh.elements().add(mesh::ElementType::Q2D4N, { 0, 1, 4, 3 }, 0, 0);
        mesh.elements().add(mesh::ElementType::Q2D4N, { 1, 2, 5, 4 }, 1, 0);
        mesh.elements().add(mesh::ElementType::Q2D4N, { 2, 6, 7, 5 }, 2, 1);

        fe_mesh.initialize(std::move(mesh));
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        mesh::Mesh mesh;

        mesh.nodes().add({ 3, 0, 0 }, 5, 1);
        mesh.nodes().add({ 4, 0, 0 }, 6, 1);
        mesh.nodes().add({ 2, 1, 0 }, 7, 1);
        mesh.nodes().add({ 3, 1, 0 }, 8, 1);
        mesh.nodes().add({ 4, 1, 0 }, 9, 1);
        mesh.nodes().add({ 2, 0, 0 }, 2, 0);
        mesh.nodes().add({ 1, 0, 0 }, 1, 0);
        mesh.nodes().add({ 1, 1, 0 }, 4, 0);

        mesh.elements().add(mesh::ElementType::Q2D4N, { 5, 0, 3, 2 }, 2, 1);
        mesh.elements().add(mesh::ElementType::Q2D4N, { 0, 1, 4, 3 }, 3, 1);
        mesh.elements().add(mesh::ElementType::Q2D4N, { 6, 5, 2, 7 }, 1, 0);

        fe_mesh.initialize(std::move(mesh));
    }
    

    mesh::ElementAdjacencyList adjacency_list;
    adjacency_list.update(fe_mesh.mesh);

    ElementScatterMap element_scatter;
    element_scatter.update(fe_mesh, adjacency_list);

    DofManager dof_manager;
    if (utility::MPIUtils::get_local_rank() == 0)
    {
        dof_manager.set_custom_enumeration(
            2,
            0,
            4,
            {
                { 0, 1, 4, 3 },
                { 1, 2, 5, 4 },
                { 2, 6, 7, 5 }
            },
            { 0, 1, 2, 3, 4, 7, 5, 8 });
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        dof_manager.set_custom_enumeration(
            2,
            5,
            9,
            {
                { 5, 0, 3, 2 },
                { 0, 1, 4, 3 },
                { 6, 5, 2, 7 }
            },
            { 5, 6, 7, 8, 9, 2, 1, 4 });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // End of setup

    auto& dof_map = dof_manager.get_dof_maps();
    if (utility::MPIUtils::get_local_rank() == 0)
    {
        assert(dof_map.rank_to_global_map == 
            std::vector<RankDofIndex>({ 0, 1, 2, 3, 4, 7, 5, 8 }));
        assert_element_to_rank(dof_map,
            {
                { 0, 1, 4, 3 },
                { 1, 2, 5, 4 },
                { 2, 6, 7, 5 }
            });
        assert(dof_map.n_owned_dofs == 5);
        assert(dof_map.n_owned_elements == 2);
        assert(dof_map.owned_global_dof_begin == 0);
        assert(dof_map.owned_global_dof_end == 4);
        assert(dof_map.n_local_ghost_dofs == 1);
        assert(dof_map.n_exclusive_ghost_dofs == 2);
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        assert(dof_map.rank_to_global_map ==
            std::vector<RankDofIndex>({ 5, 6, 7, 8, 9, 2, 1, 4 }));
        assert_element_to_rank(dof_map,
            {
                { 5, 0, 3, 2 },
                { 0, 1, 4, 3 },
                { 6, 5, 2, 7 }
            });
        assert(dof_map.n_owned_dofs == 5);
        assert(dof_map.n_owned_elements == 2);
        assert(dof_map.owned_global_dof_begin == 5);
        assert(dof_map.owned_global_dof_end == 9);
        assert(dof_map.n_local_ghost_dofs == 1);
        assert(dof_map.n_exclusive_ghost_dofs == 2);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // Enrichment

    // Enrich elements 0 and 1 as if a compatible enrichment surface cut through them on rank 0
    // Enrich element 2 as if a crack initiated in it
    // Combine DoF 4 into new enriched DoF (both are owned by rank 0 but appear as ghosts for 1)
    // Combine DoF 7 into the DoF created on 0 (owned by different ranks)

    auto& enrichment_data = dof_manager.get_enrichment_data();
    if (utility::MPIUtils::get_local_rank() == 0)
    {
        dof_manager.add_new_dof(0);
        dof_manager.add_new_dof(1);
        dof_manager.add_new_dof(4);
        dof_manager.add_new_dof(3);
        dof_manager.add_new_dof(2);
        dof_manager.add_new_dof(5);

        dof_manager.queue_dof_combination(4, 10);
        dof_manager.queue_dof_combination(5, 13);

        dof_manager.enrich_element(0, { 0, 1, 4, 3, 8, 9, 10, 11 });
        dof_manager.enrich_element(1, { 1, 2, 5, 4, 9, 12, 13, 10 });

        assert(enrichment_data.new_dof_to_old_rank_map ==
            std::vector<RankDofIndex>({ 0, 1, 4, 3, 2, 5 }));
        assert_element_to_rank(dof_map,
            {
                { 0, 1, 4, 3, 8, 9, 10, 11 },
                { 1, 2, 5, 4, 9, 12, 13, 10 },
                { 2, 6, 7, 5 }
            });
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        dof_manager.add_new_dof(5);
        dof_manager.add_new_dof(0);
        dof_manager.add_new_dof(3);
        dof_manager.add_new_dof(2);

        dof_manager.enrich_element(0, { 5, 0, 3, 2, 8, 9, 10, 11 });

        assert(enrichment_data.new_dof_to_old_rank_map ==
            std::vector<RankDofIndex>({ 5, 0, 3, 2 }));
        assert_element_to_rank(dof_map,
            {
                { 5, 0, 3, 2, 8, 9, 10, 11 },
                { 0, 1, 4, 3 },
                { 6, 5, 2, 7 }
            });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // Step 1

    auto dof_map_copy = dof_manager.get_dof_maps_copy();
    auto enrichment_data_copy = dof_manager.get_enrichment_data_copy();

    auto update_maps = detail::create_maps_for_local_enrichment_and_combinations(
        dof_map_copy,
        enrichment_data_copy);

    if (utility::MPIUtils::get_local_rank() == 0)
    {
        assert(update_maps.new_rank_to_pre_enrichment_rank_map ==
            std::vector<RankDofIndex>({ 0, 1, 2, 3, 0, 1, 4, 3, 2, 5, 5, 6, 7 }));
        assert(update_maps.post_enrichment_rank_to_new_rank_map ==
            std::vector<RankDofIndex>({ 0, 1, 2, 3, 6, 10, 11, 12, 4, 5, 6, 7, 8, 9 }));
        assert(update_maps.delta_n_dofs_by_rank[0] == 5);
        assert(update_maps.delta_n_dofs_by_rank[1] == 4);
        assert(update_maps.owned_dofs_to_combine.size() == 1);
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        assert(update_maps.new_rank_to_pre_enrichment_rank_map ==
            std::vector<RankDofIndex>({ 0, 1, 2, 3, 4, 5, 0, 3, 2, 5, 6, 7 }));
        assert(update_maps.post_enrichment_rank_to_new_rank_map ==
            std::vector<RankDofIndex>({ 0, 1, 2, 3, 4, 9, 10, 11, 5, 6, 7, 8 }));
        assert(update_maps.delta_n_dofs_by_rank[0] == 5);
        assert(update_maps.delta_n_dofs_by_rank[1] == 4);
        assert(update_maps.owned_dofs_to_combine.size() == 0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // Step 2

    auto renumbering_data = detail::update_owned_global_numbering(update_maps, dof_map_copy);

    if (utility::MPIUtils::get_local_rank() == 0)
    {
        assert(dof_map_copy.rank_to_global_map ==
            std::vector<RankDofIndex>({ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 }));

        std::vector<DofRenumberingSource> correct_source_map = {
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 0 }, // 0
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 1 }, // 1
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 2 }, // 2
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 3 }, // 3
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 0 }, // 4
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 1 }, // 5
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 4 }, // 6
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 3 }, // 7
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 2 }, // 8
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 5 } // 9
        };
        assert(renumbering_data.owned_renumbering_sources == correct_source_map);
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        assert(dof_map_copy.rank_to_global_map ==
            std::vector<RankDofIndex>({ 10, 11, 12, 13, 14, 15, 16, 17, 18 }));

        std::vector<DofRenumberingSource> correct_source_map = {
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 0 }, // 0
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 1 }, // 1
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 2 }, // 2
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 3 }, // 3
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 4 }, // 4
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 5 }, // 5
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 0 }, // 6
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 3 }, // 7
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 2 } // 8
        };
        assert(renumbering_data.owned_renumbering_sources == correct_source_map);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // Step 3

    detail::update_ghost_pre_enrichment_global_numbering(
        element_scatter,
        renumbering_data,
        update_maps,
        dof_map_copy);

    if (utility::MPIUtils::get_local_rank() == 0)
    {
        assert(dof_map_copy.rank_to_global_map ==
            std::vector<RankDofIndex>({ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 10, 13 }));
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        assert(dof_map_copy.rank_to_global_map ==
            std::vector<RankDofIndex>({ 10, 11, 12, 13, 14, 15, 16, 17, 18, 2, 1, 6 }));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // Step 4

    detail::exchange_new_exclusive_ghost_dofs(
        element_scatter,
        renumbering_data, update_maps,
        dof_map_copy);

    if (utility::MPIUtils::get_local_rank() == 0)
    {
        assert(dof_map_copy.rank_to_global_map ==
            std::vector<RankDofIndex>({ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 10, 13, 15, 16, 17, 18 }));
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        assert(dof_map_copy.rank_to_global_map ==
            std::vector<RankDofIndex>({ 10, 11, 12, 13, 14, 15, 16, 17, 18, 2, 1, 6, 5, 8, 9 }));
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    // Step 5

    detail::update_element_to_rank_map(
        update_maps,
        dof_map_copy);

    if (utility::MPIUtils::get_local_rank() == 0)
    {
        assert_element_to_rank(dof_map_copy,
            {
                { 0, 1, 6, 3, 4, 5, 6, 7 },
                { 1, 2, 10, 6, 5, 8, 9, 6 },
                { 2, 11, 12, 10 }
            });
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        assert_element_to_rank(dof_map_copy,
            {
                { 9, 0, 3, 2, 5, 6, 7, 8 },
                { 0, 1, 4, 3 },
                { 10, 9, 2, 11 }
            });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // Step 6

    auto global_to_rank_map = detail::get_global_to_rank_map(dof_map_copy);

    if (utility::MPIUtils::get_local_rank() == 0)
    {
        std::unordered_map<GlobalDofIndex, RankDofIndex> correct_map = {
            { 0, 0 },
            { 1, 1 },
            { 2, 2 },
            { 3, 3 },
            { 4, 4 },
            { 5, 5 },
            { 6, 6 },
            { 7, 7 },
            { 8, 8 },
            { 9, 9 },
            { 12, 10 },
            { 10, 11 },
            { 13, 12 },
            { 15, 13 },
            { 16, 14 },
            { 17, 15 },
            { 18, 16 }
        };
        assert(global_to_rank_map == correct_map);
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        std::unordered_map<GlobalDofIndex, RankDofIndex> correct_map = {
            { 10, 0 },
            { 11, 1 },
            { 12, 2 },
            { 13, 3 },
            { 14, 4 },
            { 15, 5 },
            { 16, 6 },
            { 17, 7 },
            { 18, 8 },
            { 2, 9 },
            { 1, 10 },
            { 6, 11 },
            { 5, 12 },
            { 8, 13 },
            { 9, 14 }
        };
        assert(global_to_rank_map == correct_map);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // Step 7

    detail::exchange_enriched_ghost_elements(
        enrichment_data_copy,
        element_scatter,
        global_to_rank_map,
        dof_map_copy);

    if (utility::MPIUtils::get_local_rank() == 0)
    {
        assert_element_to_rank(dof_map_copy,
            {
                { 0, 1, 6, 3, 4, 5, 6, 7 },
                { 1, 2, 10, 6, 5, 8, 9, 6 },
                { 2, 11, 12, 10, 13, 14, 15, 16 }
            });
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        assert_element_to_rank(dof_map_copy,
            {
                { 9, 0, 3, 2, 5, 6, 7, 8 },
                { 0, 1, 4, 3 },
                { 10, 9, 2, 11, 12, 13, 14, 11 }
            });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // Step 8

    detail::resolve_inter_rank_combinations(
        enrichment_data_copy,
        global_to_rank_map,
        update_maps,
        renumbering_data,
        dof_map_copy);

    if (utility::MPIUtils::get_local_rank() == 0)
    {
        assert(dof_map_copy.rank_to_global_map ==
            std::vector<RankDofIndex>({ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 15, 16, 17 }));

        assert_element_to_rank(dof_map_copy,
            {
                { 0, 1, 6, 3, 4, 5, 6, 7 },
                { 1, 2, 9, 6, 5, 8, 9, 6 },
                { 2, 10, 11, 9, 12, 13, 14, 15 }
            });
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        assert(dof_map_copy.rank_to_global_map ==
            std::vector<RankDofIndex>({ 10, 11, 12, 13, 14, 15, 16, 17, 2, 1, 6, 5, 8, 9 }));

        assert_element_to_rank(dof_map_copy,
            {
                { 8, 0, 2, 13, 4, 5, 6, 7 },
                { 0, 1, 3, 2 },
                { 9, 8, 13, 10, 11, 12, 13, 10 }
            });
    }


    //////////////////////////////////////////////////////////////////////////////////////////////
    // Step 9

    auto backward_permutation_indices = detail::sort_intervals(dof_map_copy);

    if (utility::MPIUtils::get_local_rank() == 0)
    {
        assert(dof_map_copy.rank_to_global_map ==
            std::vector<RankDofIndex>({ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 15, 16, 17 }));

        assert_element_to_rank(dof_map_copy,
            {
                { 0, 1, 6, 3, 4, 5, 6, 7 },
                { 1, 2, 9, 6, 5, 8, 9, 6 },
                { 2, 10, 11, 9, 12, 13, 14, 15 }
            });
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        assert(dof_map_copy.rank_to_global_map ==
            std::vector<RankDofIndex>({ 10, 11, 12, 13, 14, 15, 16, 17, 2, 9, 1, 5, 6, 8 }));

        assert_element_to_rank(dof_map_copy,
            {
                { 8, 0, 2, 9, 4, 5, 6, 7 },
                { 0, 1, 3, 2 },
                { 10, 8, 9, 12, 11, 13, 9, 12 }
            });
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    // Step 9.5

    renumbering_data.permute_owned_renumbering_sources(
        static_cast<std::size_t>(dof_map_copy.n_owned_dofs),
        backward_permutation_indices,
        detail::get_inverse_map(backward_permutation_indices));

    if (utility::MPIUtils::get_local_rank() == 0)
    {
        std::vector<DofRenumberingSource> correct_source_map = {
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 0 }, // 0
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 1 }, // 1
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 2 }, // 2
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 3 }, // 3
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 0 }, // 4
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 1 }, // 5
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 4 }, // 6
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 3 }, // 7
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 2 }, // 8
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 5 } // 9
        };
        assert(renumbering_data.owned_renumbering_sources == correct_source_map);
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        std::vector<DofRenumberingSource> correct_source_map = {
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 0 }, // 0
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 1 }, // 1
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 3 }, // 2
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 4 }, // 3
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 5 }, // 4
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 0 }, // 5
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 3 }, // 6
            { DofRenumberingSource::COPY_OLD_LOCAL_VALUE, 2 } // 7
        };
        assert(renumbering_data.owned_renumbering_sources == correct_source_map);
    }

    return 0;
}