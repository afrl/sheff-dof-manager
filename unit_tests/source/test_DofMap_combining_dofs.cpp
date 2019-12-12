#include "UnitTests.h"

using namespace dof_manager;

int main(int argc, char** argv)
{
    utility::MPIUtils::init(&argc, &argv);

    UnitTestDataStructures test_data({ 1, 1, 1 });

    DofMap dof_map(test_data.mesh);

    const auto n_dofs_unenriched = static_cast<RankDofIndex>(3 * test_data.mesh.nodes().size());
    assert(dof_map.get_n_global_dofs() == n_dofs_unenriched);
    assert(dof_map.get_n_owned_dofs() == dof_map.get_n_global_dofs());
    assert(dof_map.get_rank_to_global_map().size() == n_dofs_unenriched);
    auto orig_rank_to_global_map = dof_map.get_rank_to_global_map();
    const auto orig_n_owned_dofs = dof_map.get_n_owned_dofs();

    std::cout << "Original dof map:\n" << dof_map.str() << std::endl;

    dof_map.start_enrichment_phase();

    // Duplicate each dof
    std::vector<RankDofIndex> new_element_dofs;
    for (auto node_index : utility::range_from_zero(test_data.mesh.nodes().size()))
    {
        for (auto j : utility::range_from_zero(3))
        {
            new_element_dofs.push_back(j + 3 * node_index);
        }
    }
    for (auto node_index : utility::range_from_zero(test_data.mesh.nodes().size()))
    {
        std::vector<RankDofIndex> node_dofs;
        for (auto j : utility::range_from_zero(3))
        {
            auto new_dof = dof_map.add_new_dof(j + 3 * node_index);
            assert(new_dof == orig_n_owned_dofs + j + 3 * node_index);
            node_dofs.push_back(new_dof);
            new_element_dofs.push_back(new_dof);
        }
        dof_map.enrich_node(node_index, node_dofs.front(), node_dofs.size());
    }
    dof_map.enrich_element(0, new_element_dofs);

    // Insert combinations:
    // 0->24
    // 1->25
    // 2->26
    for (auto j : utility::range_from_zero(3))
    {
        dof_map.combine_dofs(j, orig_n_owned_dofs + j);

        auto result = dof_map.is_dof_combined_into_another(j);
        assert(result.first && result.second == orig_n_owned_dofs + j);
        result = dof_map.is_dof_combined_into_another(orig_n_owned_dofs + j);
        assert(!result.first);
    }

    // Expected result
    // Current set    Add                         Resulting set
    // 0->24          0->27 => 27->0 => 27->24    0->24
    // 1->25                                      1->25
    // 2->26                                      2->26
    //                                            27->24
    dof_map.combine_dofs(0, 27);
    auto result = dof_map.is_dof_combined_into_another(27);
    assert(result.first && result.second == 24);

    // Expected result
    // Current set    Add        Resulting set
    // 0->24          24->25     0->25
    // 1->25                     1->25
    // 2->26                     2->26
    // 27->24                    27->25
    //                           24->25
    dof_map.combine_dofs(24, 25);
    result = dof_map.is_dof_combined_into_another(0);
    assert(result.first && result.second == 25);
    result = dof_map.is_dof_combined_into_another(1);
    assert(result.first && result.second == 25);
    result = dof_map.is_dof_combined_into_another(2);
    assert(result.first && result.second == 26);
    result = dof_map.is_dof_combined_into_another(27);
    assert(result.first && result.second == 25);
    result = dof_map.is_dof_combined_into_another(24);
    assert(result.first && result.second == 25);

    // Expected result
    // Current set    Add              Resulting set
    // 0->25          2->1 => 2->25    0->25
    // 1->25                 26->25    1->25
    // 2->26                           2->25
    // 27->25                          27->25
    // 24->25                          24->25
    //                                 26->25
    dof_map.combine_dofs(2, 1);
    result = dof_map.is_dof_combined_into_another(0);
    assert(result.first && result.second == 25);
    result = dof_map.is_dof_combined_into_another(1);
    assert(result.first && result.second == 25);
    result = dof_map.is_dof_combined_into_another(2);
    assert(result.first && result.second == 25);
    result = dof_map.is_dof_combined_into_another(27);
    assert(result.first && result.second == 25);
    result = dof_map.is_dof_combined_into_another(24);
    assert(result.first && result.second == 25);
    result = dof_map.is_dof_combined_into_another(26);
    assert(result.first && result.second == 25);

    dof_map.start_renumber_phase();
    dof_map.end_renumber_phase();

    std::cout << dof_map.str() << std::endl;

    return 0;
}