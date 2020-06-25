#include "dof_manager/RenumberingAlgorithm.h"
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
    for (auto i : utility::range_from_zero(element_to_rank.size()))
    {
        assert(element_to_rank[i].size() == dof_map.element_n_dofs[i]);
        for (auto j : utility::range_from_zero(element_to_rank[i].size()))
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
    MeshType mesh;
    if (utility::MPIUtils::get_local_rank() == 0)
    {
        auto n0 = mesh.add_node_by_coord({ 0, 0, 0 });
        n0->set_local_number(0);
        n0->set_global_number(0);
        n0->set_parent_partition(0);

        auto n1 = mesh.add_node_by_coord({ 1, 0, 0 });
        n1->set_local_number(1);
        n1->set_global_number(1);
        n1->set_parent_partition(0);

        auto n2 = mesh.add_node_by_coord({ 2, 0, 0 });
        n2->set_local_number(2);
        n2->set_global_number(2);
        n2->set_parent_partition(0);

        auto n3 = mesh.add_node_by_coord({ 0, 1, 0 });
        n3->set_local_number(3);
        n3->set_global_number(3);
        n3->set_parent_partition(0);

        auto n4 = mesh.add_node_by_coord({ 1, 1, 0 });
        n4->set_local_number(4);
        n4->set_global_number(4);
        n4->set_parent_partition(0);

        auto n7 = mesh.add_node_by_coord({ 2, 1, 0 });
        n7->set_local_number(5);
        n7->set_global_number(7);
        n7->set_parent_partition(1);

        auto n5 = mesh.add_node_by_coord({ 3, 0, 0 });
        n5->set_local_number(6);
        n5->set_global_number(5);
        n5->set_parent_partition(1);

        auto n8 = mesh.add_node_by_coord({ 3, 1, 0 });
        n8->set_local_number(7);
        n8->set_global_number(8);
        n8->set_parent_partition(1);

        auto e0 = mesh.elements().add();
        e0->initialize(flex_mesh::ElementType::Q2D4N, { n0, n1, n4, n3 });
        e0->set_local_number(0);
        e0->set_global_number(0);
        e0->set_parent_partition(0);

        auto e1 = mesh.elements().add();
        e1->initialize(flex_mesh::ElementType::Q2D4N, { n1, n2, n7, n4 });
        e1->set_local_number(1);
        e1->set_global_number(1);
        e1->set_parent_partition(0);

        auto e2 = mesh.elements().add();
        e2->initialize(flex_mesh::ElementType::Q2D4N, { n2, n5, n8, n7 });
        e2->set_local_number(2);
        e2->set_global_number(2);
        e2->set_parent_partition(1);
    }
    else if (utility::MPIUtils::get_local_rank() == 1)
    {
        auto n5 = mesh.add_node_by_coord({ 3, 0, 0 });
        n5->set_local_number(0);
        n5->set_global_number(5);
        n5->set_parent_partition(1);

        auto n6 = mesh.add_node_by_coord({ 4, 0, 0 });
        n6->set_local_number(1);
        n6->set_global_number(6);
        n6->set_parent_partition(1);

        auto n7 = mesh.add_node_by_coord({ 2, 1, 0 });
        n7->set_local_number(2);
        n7->set_global_number(7);
        n7->set_parent_partition(1);

        auto n8 = mesh.add_node_by_coord({ 3, 1, 0 });
        n8->set_local_number(3);
        n8->set_global_number(8);
        n8->set_parent_partition(1);

        auto n9 = mesh.add_node_by_coord({ 4, 1, 0 });
        n9->set_local_number(4);
        n9->set_global_number(9);
        n9->set_parent_partition(1);

        auto n2 = mesh.add_node_by_coord({ 2, 0, 0 });
        n2->set_local_number(5);
        n2->set_global_number(2);
        n2->set_parent_partition(0);

        auto n1 = mesh.add_node_by_coord({ 1, 0, 0 });
        n1->set_local_number(6);
        n1->set_global_number(1);
        n1->set_parent_partition(0);

        auto n4 = mesh.add_node_by_coord({ 1, 1, 0 });
        n4->set_local_number(7);
        n4->set_global_number(4);
        n4->set_parent_partition(0);

        auto e2 = mesh.elements().add();
        e2->initialize(flex_mesh::ElementType::Q2D4N, { n2, n5, n8, n7 });
        e2->set_local_number(0);
        e2->set_global_number(2);
        e2->set_parent_partition(1);

        auto e3 = mesh.elements().add();
        e3->initialize(flex_mesh::ElementType::Q2D4N, { n5, n6, n9, n8 });
        e3->set_local_number(1);
        e3->set_global_number(3);
        e3->set_parent_partition(1);

        auto e1 = mesh.elements().add();
        e1->initialize(flex_mesh::ElementType::Q2D4N, { n1, n2, n7, n4 });
        e1->set_local_number(2);
        e1->set_global_number(1);
        e1->set_parent_partition(0);
    }

    ElementAdjacencyList adjacency_list;
    adjacency_list.initialize(mesh);

    ElementScatterMap element_scatter(mesh, adjacency_list);

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