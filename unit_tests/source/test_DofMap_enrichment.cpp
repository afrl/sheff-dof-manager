#include "UnitTests.h"

using namespace dof_manager;

int main(int argc, char** argv)
{
    utility::MPIUtils::init(&argc, &argv);

    UnitTestDataStructures test_data({ 1, 1, 1 });

    mesh::ElementAdjacencyList adjacency_list;
    adjacency_list.update(test_data.fe_mesh.mesh);

    ElementScatterMap element_scatter;
    element_scatter.update(test_data.fe_mesh, adjacency_list);

    DofManager dof_manager;
    dof_manager.enumerate(test_data.fe_mesh, element_scatter);
    auto& dof_map = dof_manager.get_dof_maps();
    auto& enrichment_data = dof_manager.get_enrichment_data();

    const auto n_dofs_unenriched =
      static_cast<RankDofIndex>(3 * test_data.fe_mesh.mesh.nodes().size());
    assert(dof_map.n_owned_dofs == n_dofs_unenriched);
    assert(dof_map.rank_to_global_map.size() == n_dofs_unenriched);
    auto orig_rank_to_global_map = dof_map.rank_to_global_map;

    auto element_map = dof_map.get_element_map(0);
    assert(element_map.size() == 3 * test_data.fe_mesh.mesh.elements()[0].connectivity().size());
    // Make sure the set is unique for the element
    std::set<RankDofIndex> dof_set(element_map.begin(), element_map.end());
    assert(dof_set.size() == 3 * test_data.fe_mesh.mesh.elements()[0].connectivity().size());

    auto new_dof = dof_manager.add_new_dof(0);
    std::vector<RankDofIndex> new_element_dofs(element_map.begin(), element_map.end());
    new_element_dofs.push_back(new_dof);
    dof_manager.enrich_element(0, new_element_dofs);
    auto& new_dof_to_old_rank_map = enrichment_data.new_dof_to_old_rank_map;
    assert(new_dof_to_old_rank_map.size() == 1);
    assert(new_dof_to_old_rank_map[0] == 0);

    auto renumber_data = dof_manager.renumber(element_scatter);
    for (auto i : utility::range(new_dof - 1))
    {
        assert(renumber_data.owned_renumbering_sources[i].source_dof_rank_index == i);
    }
    assert(renumber_data.owned_renumbering_sources[new_dof].source_dof_rank_index == 0);
    assert(renumber_data.old_rank_to_global_map == orig_rank_to_global_map);

    assert(dof_map.n_owned_dofs == n_dofs_unenriched + 1);
    assert(dof_map.rank_to_global_map.size() == dof_map.n_owned_dofs);

    return 0;
}