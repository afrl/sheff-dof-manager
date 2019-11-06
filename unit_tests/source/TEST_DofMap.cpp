#include "UnitTests.h"

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

    auto element_map = dof_map.get_element_map(0);
    assert(element_map.size() == 3 * test_data.mesh.elements()[0]->get_num_nodes());
    // Make sure the set is unique for the element
    std::set<RankDofIndex> dof_set(element_map.begin(), element_map.end());
    assert(dof_set.size() == 3 * test_data.mesh.elements()[0]->get_num_nodes());

    // Make sure these throw errors
    _UTIL_ASSERT_THROW(dof_map.add_new_dof(0));
    _UTIL_ASSERT_THROW(dof_map.enrich_element(0, {}));
    _UTIL_ASSERT_THROW(dof_map.enrich_node(0, 0, 1));
    _UTIL_ASSERT_THROW(dof_map.get_renumbering_new_rank_to_old_rank_map());
    _UTIL_ASSERT_THROW(dof_map.get_renumbering_old_rank_to_global_map());

    dof_map.start_enrichment_phase();
    _UTIL_ASSERT_THROW(dof_map.enrich_element(0, { n_dofs_unenriched }));
    _UTIL_ASSERT_THROW(dof_map.enrich_node(0, n_dofs_unenriched, 1));
    auto new_dof = dof_map.add_new_dof(0);
    std::vector<RankDofIndex> new_element_dofs(element_map.begin(), element_map.end());
    new_element_dofs.push_back(new_dof);
    dof_map.enrich_element(0, new_element_dofs);
    dof_map.enrich_node(0, new_dof, 1);
    auto& new_dof_to_old_rank_map = dof_map.get_enrichment_new_dof_to_old_rank_map();
    assert(new_dof_to_old_rank_map.size() == 1);
    assert(new_dof_to_old_rank_map[0] == 0);
    _UTIL_ASSERT_THROW(dof_map.get_renumbering_new_rank_to_old_rank_map());
    _UTIL_ASSERT_THROW(dof_map.get_renumbering_old_rank_to_global_map());

    dof_map.start_renumber_phase();
    _UTIL_ASSERT_THROW(dof_map.add_new_dof(0));
    _UTIL_ASSERT_THROW(dof_map.enrich_element(0, {}));
    _UTIL_ASSERT_THROW(dof_map.enrich_node(0, 0, 1));
    auto& new_rank_to_old_rank_map = dof_map.get_renumbering_new_rank_to_old_rank_map();
    for (auto i : utility::range_from_zero(new_dof - 1))
    {
        assert(new_rank_to_old_rank_map[i] == i);
    }
    assert(new_rank_to_old_rank_map[new_dof] == 0);
    assert(dof_map.get_renumbering_old_rank_to_global_map() == orig_rank_to_global_map);

    dof_map.end_renumber_phase();
    _UTIL_ASSERT_THROW(dof_map.add_new_dof(0));
    _UTIL_ASSERT_THROW(dof_map.enrich_element(0, {}));
    _UTIL_ASSERT_THROW(dof_map.enrich_node(0, 0, 1));
    _UTIL_ASSERT_THROW(dof_map.get_renumbering_new_rank_to_old_rank_map());
    _UTIL_ASSERT_THROW(dof_map.get_renumbering_old_rank_to_global_map());

    assert(dof_map.get_n_global_dofs() == n_dofs_unenriched + 1);
    assert(dof_map.get_n_owned_dofs() == dof_map.get_n_global_dofs());
    assert(dof_map.get_rank_to_global_map().size() == dof_map.get_n_global_dofs());

    return 0;
}