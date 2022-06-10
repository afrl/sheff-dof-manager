#include "dof_manager/detail/BasalFieldCompatibilityMap.h"

#include "../rapid_xml/rapidxml.hpp"
#include "../rapid_xml/rapidxml_print.hpp"
#include "finite_element/hhe_primitives/EnrichmentTreeTraverser.h"
#include "finite_element/hhe_primitives/PrimitiveAlgorithms.h"

#include <fstream>
#include <iostream>

namespace dof_manager
{
namespace detail
{

/// @brief Adds a symmetric compatibility for (element_index_1, basal_field_index_1) <=>
///        (element_index_2, basal_field_index_2).
static void add_symmetric_entry(
  std::vector<utility::DynamicArray<BasalFieldCompatibility, int16_t>>& list,
  const mesh::LocalIndex element_index_1,
  const finite_element::LeafIndex basal_field_index_1,
  const mesh::LocalIndex element_index_2,
  const finite_element::LeafIndex basal_field_index_2)
{
    list[element_index_1].push_back({ basal_field_index_1, element_index_2, basal_field_index_2 });
    list[element_index_2].push_back({ basal_field_index_2, element_index_1, basal_field_index_1 });
}

/// @brief Removes all compatibility entries that involve the given element (includes removal of
/// symmetric entries in adjacent elements).
static void remove_all_compatibilties_for_element(
  std::vector<utility::DynamicArray<BasalFieldCompatibility, int16_t>>& list,
  const mesh::LocalIndex element_index)
{
    std::unordered_set<mesh::LocalIndex> neighbor_elements_to_fix;
    for (auto& compatibility : list[element_index])
    {
        neighbor_elements_to_fix.insert(compatibility.neighbor_element_index);
    }
    for (auto& neighbor_element_index : neighbor_elements_to_fix)
    {
        utility::DynamicArray<BasalFieldCompatibility, int16_t> compatibilities;
        for (auto& compatibility : list[neighbor_element_index])
        {
            if (compatibility.neighbor_element_index != element_index)
            {
                compatibilities.push_back(compatibility);
            }
        }
        list[neighbor_element_index].swap(compatibilities);
    }
    list[element_index].clear();
}

/// @brief Kernel for creating compatibilities between basal fields in a given element and those in
///        its neighbors.
/// @param element_update_flags - a bool for each element indicating if the respective element is
///        being updated with new compatibilities.
static void create_compatibilities_with_neighbors(
  BasalFieldCompatibilityMap& map,
  const finite_element::FiniteElementMesh& fe_mesh,
  const mesh::ElementAdjacencyList& element_adjacency,
  const mesh::LocalIndex& element_index,
  const std::vector<bool>& element_update_flags)
{
    auto fe            = fe_mesh.get_finite_element(element_index);
    auto& this_element = fe.element;
    finite_element::EnrichmentTreeTraverser this_tree_traverser(fe);

    for (auto& neighbor_element :
         element_adjacency.get_adjacent_elements(this_element.local_index()))
    {
        // Since compatibility entries are symmetric, to avoid duplicate compatibility entries, the
        // relationship will be processed for the element with the lower local index of two given
        // elements.  However, if the neighbor element is not to be updated, then process it.
        if (this_element.local_index() < neighbor_element.local_index() ||
            !element_update_flags[neighbor_element.local_index()])
        {
            auto neighbor_fe = fe_mesh.get_finite_element(neighbor_element.local_index());

            // Short-circuit the logic if both are not enriched
            if (fe.layout.get_n_enrichments() == 0 && neighbor_fe.layout.get_n_enrichments() == 0)
            {
                add_symmetric_entry(map.compatibilities,
                                    this_element.local_index(),
                                    0,
                                    neighbor_element.local_index(),
                                    0);
            }
            else
            {
                auto shared_node_positions =
                  mesh::find_shared_node_positions(this_element, neighbor_element);
                auto this_shared_topology =
                  mesh::find_shared_topology_component(this_element, true, shared_node_positions);
                auto neighbor_shared_topology =
                  mesh::find_shared_topology_component(neighbor_element,
                                                       false,
                                                       shared_node_positions);

                std::vector<finite_element::LeafIndex> this_basal_fields_to_search;
                for (auto basal_index : utility::range(fe.layout.get_n_basal_fields()))
                {
                    auto field_touches_shared_topology =
                      finite_element::does_basal_field_touch_topology(fe,
                                                                      basal_index,
                                                                      this_shared_topology);
                    if (field_touches_shared_topology)
                    {
                        this_basal_fields_to_search.push_back(basal_index);
                    }
                }


                std::vector<finite_element::LeafIndex> neighbor_basal_fields_to_search;
                for (auto basal_index : utility::range(neighbor_fe.layout.get_n_basal_fields()))
                {
                    auto field_touches_shared_topology =
                      finite_element::does_basal_field_touch_topology(neighbor_fe,
                                                                      basal_index,
                                                                      neighbor_shared_topology);
                    if (field_touches_shared_topology)
                    {
                        neighbor_basal_fields_to_search.push_back(basal_index);
                    }
                }

                finite_element::EnrichmentTreeTraverser neighbor_tree_traverser(neighbor_fe);

                for (auto& this_basal_field_index : this_basal_fields_to_search)
                {
                    for (auto& neighbor_basal_field_index : neighbor_basal_fields_to_search)
                    {
                        auto& this_basal_field = fe.layout.get_basal_field(this_basal_field_index);
                        auto& neighbor_basal_field =
                          neighbor_fe.layout.get_basal_field(neighbor_basal_field_index);

                        auto compatibility_type =
                          finite_element::are_leaves_compatible(this_tree_traverser,
                                                                this_basal_field_index,
                                                                this_shared_topology,
                                                                neighbor_tree_traverser,
                                                                neighbor_basal_field_index,
                                                                neighbor_shared_topology);
                        if (compatibility_type == finite_element::LeafCompatibilityType::ONE_TO_ONE)
                        {
                            add_symmetric_entry(map.compatibilities,
                                                this_element.local_index(),
                                                this_basal_field_index,
                                                neighbor_element.local_index(),
                                                neighbor_basal_field_index);
                        }
                        else if (compatibility_type == finite_element::LeafCompatibilityType::MANY)
                        {
                            add_symmetric_entry(map.incompatibilities,
                                                this_element.local_index(),
                                                this_basal_field_index,
                                                neighbor_element.local_index(),
                                                neighbor_basal_field_index);
                        }
                    }
                }
            }
        }
    }
}

static void update_second_neighbor_compatibilities(
  BasalFieldCompatibilityMap& map,
  const finite_element::FiniteElementMesh& fe_mesh,
  const mesh::ElementAdjacencyList& element_adjacency,
  const std::vector<bool>& element_update_flags)
{
    // TODO this only needs to be updated for modified elements right?
    for (auto element_index : utility::range(fe_mesh.mesh.elements().size()))
    {
        bool requires_update = element_update_flags[element_index];
        for (auto& n : element_adjacency.get_adjacent_element_indices(element_index))
        {
            requires_update |= element_update_flags[n];
        }

        if (requires_update)
        {
            auto fe_accessor              = fe_mesh.get_finite_element(element_index);
            auto& element_compatibilities = map.compatibilities[element_index];

            // Build neighborhood adjacency matrix
            // TODO move this to the adjacency list class
            auto neighbors   = element_adjacency.get_adjacent_element_indices(element_index);
            auto n_neighbors = neighbors.size();
            simple_math::DynamicBoolMatrix neighborhood_adjacency_matrix =
              simple_math::DynamicBoolMatrix::Zero(n_neighbors + 1, n_neighbors + 1);
            std::unordered_map<mesh::LocalIndex, mesh::ConnIndex> element_to_matrix_index_map = {
                { element_index, 0 }
            };
            for (auto i : utility::range(n_neighbors))
            {
                element_to_matrix_index_map.insert(
                  { neighbors[i], static_cast<mesh::ConnIndex>(i + 1) });
                neighborhood_adjacency_matrix(0, i + 1) = true;
                neighborhood_adjacency_matrix(i + 1, 0) = true;
            }
            for (auto i : utility::range(n_neighbors))
            {
                auto row_index = i + 1;
                auto neighbors_of_neighbor =
                  element_adjacency.get_adjacent_element_indices(neighbors[i]);
                for (auto& neighbor_of_neighbor : neighbors_of_neighbor)
                {
                    auto find_iter = element_to_matrix_index_map.find(neighbor_of_neighbor);
                    if (find_iter != element_to_matrix_index_map.end())
                    {
                        auto& col_index                                     = find_iter->second;
                        neighborhood_adjacency_matrix(row_index, col_index) = true;
                    }
                }
            }
            // End build neighborhood adjacency matrix

            // TODO consider storing element index relative to adjacency instead of the mesh to
            // avoid searches
            for (auto basal_field_index : utility::range(fe_accessor.layout.get_n_basal_fields()))
            {
                std::vector<std::pair<mesh::LocalIndex, finite_element::LeafIndex>>
                  compatibilities_for_basal_field;
                for (auto& compatibility : element_compatibilities)
                {
                    auto neighbor_fe =
                      fe_mesh.get_finite_element(compatibility.neighbor_element_index);
                    // Note: only enriched elements are included since it impossible to find a new
                    // connection for non-enriched elements
                    if (compatibility.this_basal_field_index == basal_field_index &&
                        neighbor_fe.layout.get_n_basal_fields() > 1)
                    {
                        compatibilities_for_basal_field.push_back(
                          { compatibility.neighbor_element_index,
                            compatibility.neighbor_basal_field_index });
                    }
                }

                for (auto& c1 : compatibilities_for_basal_field)
                {
                    for (auto& c2 : compatibilities_for_basal_field)
                    {
                        auto& c1_compatibilities = map.compatibilities[c1.first];
                        // Test that:
                        // 1) The elements in c1 and c2 are not the same
                        // 2) The elements in c1 and c2 are adjacent to each other
                        // 3) A compatibility between c1 and c2 does not already exist
                        // 4) An incompatibility between c1 and c2 does not exist
                        if (c1.first != c2.first &&
                            neighborhood_adjacency_matrix(element_to_matrix_index_map[c1.first],
                                                          element_to_matrix_index_map[c2.first]) &&
                            !compatibility_exists(map, c1.first, c1.second, c2.first, c2.second) &&
                            !incompatibility_exists(map, c1.first, c1.second, c2.first, c2.second))
                        {
                            add_symmetric_entry(map.compatibilities,
                                                c1.first,
                                                c1.second,
                                                c2.first,
                                                c2.second);
                        }
                    }
                }
            }
        }
    }
}

/// @param fe_mesh - the corresponding mesh
/// @param map - the list of compatibilities/incompatibilities
/// @param atom_type - use C for compatibilities (to render grey) and O for incompatibilities (to
/// render red)
static void compatibility_graph_kernel(
  const std::string& filestem,
  const finite_element::FiniteElementMesh& fe_mesh,
  const std::vector<utility::DynamicArray<BasalFieldCompatibility, int16_t>>& map,
  const std::string& atom_type = "C")
{
    using namespace rapidxml;

    auto doc = std::make_unique<rapidxml::xml_document<>>();

    rapidxml::xml_node<>* molecule_node = doc->allocate_node(rapidxml::node_element, "molecule");
    rapidxml::xml_attribute<>* attr     = doc->allocate_attribute("title", "compatibility_map");
    molecule_node->append_attribute(attr);
    doc->append_node(molecule_node);

    rapidxml::xml_node<>* atom_array = doc->allocate_node(rapidxml::node_element, "atomArray");
    molecule_node->append_node(atom_array);

    const std::string atom_str          = "atom";
    const std::string id_str            = "id";
    const std::string elementType_str   = "elementType";
    const std::string elementType_value = atom_type;
    const std::string x3_str            = "x3";
    const std::string y3_str            = "y3";
    const std::string z3_str            = "z3";
    std::vector<std::vector<std::string>> element_field_names(fe_mesh.mesh.elements().size());
    std::vector<std::vector<std::string>> x3_values(fe_mesh.mesh.elements().size());
    std::vector<std::vector<std::string>> y3_values(fe_mesh.mesh.elements().size());
    std::vector<std::vector<std::string>> z3_values(fe_mesh.mesh.elements().size());
    for (auto& this_element : fe_mesh.mesh.elements())
    {
        auto fe = fe_mesh.get_finite_element(this_element.local_index());
        for (auto i : utility::range(fe.layout.get_n_basal_fields()))
        {
            element_field_names[this_element.local_index()].push_back(
              std::to_string(this_element.local_index()) + "_" + std::to_string(i));

            auto p = finite_element::get_point_in_basal_field(fe, i);
            x3_values[this_element.local_index()].push_back(std::to_string(p[0]));
            y3_values[this_element.local_index()].push_back(std::to_string(p[1]));
            z3_values[this_element.local_index()].push_back(std::to_string(p[2]));
        }
    }
    for (auto& this_element : fe_mesh.mesh.elements())
    {
        auto fe = fe_mesh.get_finite_element(this_element.local_index());
        for (auto i : utility::range(fe.layout.get_n_basal_fields()))
        {
            rapidxml::xml_node<>* atom_node =
              doc->allocate_node(rapidxml::node_element, atom_str.c_str(), 0, atom_str.size());
            atom_array->append_node(atom_node);

            rapidxml::xml_attribute<>* id_attr =
              doc->allocate_attribute(id_str.c_str(),
                                      element_field_names[this_element.local_index()][i].c_str(),
                                      id_str.size(),
                                      element_field_names[this_element.local_index()][i].size());
            atom_node->append_attribute(id_attr);

            rapidxml::xml_attribute<>* elementType_attr =
              doc->allocate_attribute(elementType_str.c_str(),
                                      elementType_value.c_str(),
                                      elementType_str.size(),
                                      elementType_value.size());
            atom_node->append_attribute(elementType_attr);

            rapidxml::xml_attribute<>* x3_attr =
              doc->allocate_attribute(x3_str.c_str(),
                                      x3_values[this_element.local_index()][i].c_str(),
                                      x3_str.size(),
                                      x3_values[this_element.local_index()][i].size());
            atom_node->append_attribute(x3_attr);

            rapidxml::xml_attribute<>* y3_attr =
              doc->allocate_attribute(y3_str.c_str(),
                                      y3_values[this_element.local_index()][i].c_str(),
                                      y3_str.size(),
                                      y3_values[this_element.local_index()][i].size());
            atom_node->append_attribute(y3_attr);

            rapidxml::xml_attribute<>* z3_attr =
              doc->allocate_attribute(z3_str.c_str(),
                                      z3_values[this_element.local_index()][i].c_str(),
                                      z3_str.size(),
                                      z3_values[this_element.local_index()][i].size());
            atom_node->append_attribute(z3_attr);
        }
    }

    rapidxml::xml_node<>* bond_array = doc->allocate_node(rapidxml::node_element, "bondArray");
    molecule_node->append_node(bond_array);

    const std::string bond_str = "bond";
    std::vector<std::string> bond_id_values;
    const std::string atomRefs2_str = "atomRefs2";
    std::vector<std::string> atomRefs2_values;
    const std::string order_str   = "order";
    const std::string order_value = "1";
    for (auto& this_element : fe_mesh.mesh.elements())
    {
        for (auto& c : map[this_element.local_index()])
        {
            bond_id_values.push_back("b" + std::to_string(bond_id_values.size()));
            atomRefs2_values.push_back(
              element_field_names[this_element.local_index()][c.this_basal_field_index] + " " +
              element_field_names[c.neighbor_element_index][c.neighbor_basal_field_index]);
        }
    }
    std::size_t counter = 0;
    for (auto& this_element : fe_mesh.mesh.elements())
    {
        for (auto& c : map[this_element.local_index()])
        {
            rapidxml::xml_node<>* bond_node =
              doc->allocate_node(rapidxml::node_element, bond_str.c_str(), 0, bond_str.size());
            bond_array->append_node(bond_node);

            rapidxml::xml_attribute<>* id_attr =
              doc->allocate_attribute(id_str.c_str(),
                                      bond_id_values[counter].c_str(),
                                      id_str.size(),
                                      bond_id_values[counter].size());
            bond_node->append_attribute(id_attr);

            rapidxml::xml_attribute<>* atomRefs2_attr =
              doc->allocate_attribute(atomRefs2_str.c_str(),
                                      atomRefs2_values[counter].c_str(),
                                      atomRefs2_str.size(),
                                      atomRefs2_values[counter].size());
            bond_node->append_attribute(atomRefs2_attr);

            rapidxml::xml_attribute<>* order_attr = doc->allocate_attribute(order_str.c_str(),
                                                                            order_value.c_str(),
                                                                            order_str.size(),
                                                                            order_value.size());
            bond_node->append_attribute(order_attr);

            counter++;
        }
    }

    std::ofstream out(filestem + ".cml");
    out << *doc;
    out.close();
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Defintion from header

BasalFieldCompatibilityMap build_initial_compatibity_map(
  const finite_element::FiniteElementMesh& fe_mesh,
  const mesh::ElementAdjacencyList& element_adjacency)
{
    BasalFieldCompatibilityMap map;
    map.compatibilities.resize(fe_mesh.mesh.elements().size());
    map.incompatibilities.resize(fe_mesh.mesh.elements().size());
    map.n_basal_fields_at_last_update.resize(fe_mesh.mesh.elements().size());

    std::vector<bool> element_update_flags(fe_mesh.mesh.elements().size(), true);

    bool any_element_enriched = false;
    for (auto& this_element : fe_mesh.mesh.elements())
    {
        auto fe = fe_mesh.get_finite_element(this_element.local_index());
        map.n_basal_fields_at_last_update[this_element.local_index()] =
          fe.layout.get_n_basal_fields();
        any_element_enriched |= fe.layout.get_n_enrichments() > 0;

        create_compatibilities_with_neighbors(map,
                                              fe_mesh,
                                              element_adjacency,
                                              this_element.local_index(),
                                              element_update_flags);
    }

    if (any_element_enriched)
    {
        update_second_neighbor_compatibilities(map,
                                               fe_mesh,
                                               element_adjacency,
                                               element_update_flags);
    }

    return map;
}



void update_compatibility_map(BasalFieldCompatibilityMap& map,
                              const finite_element::FiniteElementMesh& fe_mesh,
                              const mesh::ElementAdjacencyList& element_adjacency)
{
    // Remove all compatibilties for elements have been enriched since last update/build
    std::vector<bool> element_update_flags(fe_mesh.mesh.elements().size(), false);
    for (auto& this_element : fe_mesh.mesh.elements())
    {
        auto fe = fe_mesh.get_finite_element(this_element.local_index());
        if (map.n_basal_fields_at_last_update[this_element.local_index()] !=
            fe.layout.get_n_basal_fields())
        {
            element_update_flags[this_element.local_index()] = true;
            remove_all_compatibilties_for_element(map.compatibilities, this_element.local_index());
            remove_all_compatibilties_for_element(map.incompatibilities,
                                                  this_element.local_index());
        }
    }

    detail::write_compatibility_graph("tmp_compatibility_graph", fe_mesh, map);

    // Create new compatibilities between any element that was modified (new enrichments) since last
    // update/build
    for (auto& this_element : fe_mesh.mesh.elements())
    {
        if (element_update_flags[this_element.local_index()])
        {
            auto fe = fe_mesh.get_finite_element(this_element.local_index());
            map.n_basal_fields_at_last_update[this_element.local_index()] =
              fe.layout.get_n_basal_fields();

            create_compatibilities_with_neighbors(map,
                                                  fe_mesh,
                                                  element_adjacency,
                                                  this_element.local_index(),
                                                  element_update_flags);
        }
    }

    detail::write_compatibility_graph("tmp_compatibility_graph", fe_mesh, map);

    update_second_neighbor_compatibilities(map, fe_mesh, element_adjacency, element_update_flags);
}

bool compatibility_exists(BasalFieldCompatibilityMap& map,
                          const mesh::LocalIndex element_index_1,
                          const finite_element::LeafIndex basal_field_index_1,
                          const mesh::LocalIndex element_index_2,
                          const finite_element::LeafIndex basal_field_index_2)
{
    for (auto& c : map.compatibilities[element_index_1])
    {
        if (c.this_basal_field_index == basal_field_index_1 &&
            c.neighbor_element_index == element_index_2 &&
            c.neighbor_basal_field_index == basal_field_index_2)
        {
            return true;
        }
    }
    return false;
}

bool incompatibility_exists(BasalFieldCompatibilityMap& map,
                            const mesh::LocalIndex element_index_1,
                            const finite_element::LeafIndex basal_field_index_1,
                            const mesh::LocalIndex element_index_2,
                            const finite_element::LeafIndex basal_field_index_2)
{
    for (auto& c : map.incompatibilities[element_index_1])
    {
        if (c.this_basal_field_index == basal_field_index_1 &&
            c.neighbor_element_index == element_index_2 &&
            c.neighbor_basal_field_index == basal_field_index_2)
        {
            return true;
        }
    }
    return false;
}

void write_compatibility_graph(const std::string& filestem,
                               const finite_element::FiniteElementMesh& fe_mesh,
                               const BasalFieldCompatibilityMap& map)
{
    compatibility_graph_kernel(filestem, fe_mesh, map.compatibilities, "C");
}

void write_incompatibility_graph(const std::string& filestem,
                                 const finite_element::FiniteElementMesh& fe_mesh,
                                 const BasalFieldCompatibilityMap& map)
{
    compatibility_graph_kernel(filestem, fe_mesh, map.incompatibilities, "O");
}

}
}