#include "dof_manager/detail/ScatterMapDestBuilder.h"

#include "utility/MPIUtils.h"

namespace dof_manager
{
namespace detail
{

void ScatterMapDestBuilder::add_source_rank(int rank)
{
    source_ranks_bitset_.add(rank);
}

std::vector<int> ScatterMapDestBuilder::get_dest_ranks()
{
    std::vector<char> serialized_bitset(source_ranks_bitset_.getSizeInBytes());
    source_ranks_bitset_.write(serialized_bitset.data());
    std::string serialized_bitset_str(serialized_bitset.begin(), serialized_bitset.end());
    // std::cout << utility::MPIUtils::get_local_rank() << ": sending bitset: " <<
    // source_ranks_bitset_.toString() << " as '" << serialized_bitset_str << "' with size " <<
    // serialized_bitset.size() << std::endl;

    // Note: the returned value is only non-empty for the root rank
    auto source_rank_bitset_by_rank =
      utility::MPIUtils::gather_strings_by_rank(serialized_bitset_str);

    // Create a multimap that stores pairs of rank indices with:
    // * key - the rank that should send the data (to the one indicated by the value)
    // * value - the rank that requires data
    std::multimap<int, int> rank_to_source_multimap;
    for (auto rank : utility::indices(source_rank_bitset_by_rank))
    {
        roaring::Roaring source_rank_for_dest;
        source_rank_for_dest = roaring::Roaring::readSafe(source_rank_bitset_by_rank[rank].c_str(),
                                                 source_rank_bitset_by_rank[rank].size());
        // std::cout << "Serialized bit from " << rank << ": '" << source_rank_bitset_by_rank[rank]
        // <<
        //          "' with size " << source_rank_bitset_by_rank[rank].size() << " deserializes to "
        //          << source_rank_for_dest.toString() << std::endl;
        for (auto iter = source_rank_for_dest.begin(); iter != source_rank_for_dest.end(); iter++)
        {
            rank_to_source_multimap.insert({ *iter, static_cast<int>(rank) });
        }
    }

    // Contians the unrolled matrix of dest ranks for each rank
    std::vector<int> dest_rank_data;
    dest_rank_data.reserve(rank_to_source_multimap.size());
    // Offets to distinguish which part of dest_rank_data is relevant to each rank
    size_t curr_offset                    = 0;
    std::vector<size_t> dest_rank_offsets = { curr_offset };
    for (auto rank : utility::indices(source_rank_bitset_by_rank))
    {
        // std::cout << "Sending dest's to rank " << rank;
        // Gets all of the values in the multimap for the current rank is the key
        auto iter_range = rank_to_source_multimap.equal_range(static_cast<int>(rank));
        for (auto iter = iter_range.first; iter != iter_range.second; iter++)
        {
            // std::cout << iter->second << " ";
            dest_rank_data.push_back(iter->second);
            curr_offset++;
        }
        // std::cout << std::endl;
        dest_rank_offsets.push_back(curr_offset);
    }

    auto dest_ranks = utility::MPIUtils::scatter_arrays(dest_rank_data, dest_rank_offsets);

    std::sort(dest_ranks.begin(), dest_ranks.end());

    return dest_ranks;
}

std::vector<int> ScatterMapDestBuilder::get_source_ranks()
{
    std::vector<int> source_ranks;
    for (auto iter = source_ranks_bitset_.begin(); iter != source_ranks_bitset_.end(); iter++)
    {
        source_ranks.push_back(*iter);
    }

    std::sort(source_ranks.begin(), source_ranks.end());

    return source_ranks;
}

}
}