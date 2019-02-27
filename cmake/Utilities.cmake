# For targets that use this project they should call this
# in order to group the files by the directories they are
# in within Visual Studio.
function(assign_source_group _include_match_name)
    foreach(_source IN ITEMS ${ARGN})
		# Since CMake does not support look ahead and look behind expressions
		# We will replace "${_include_match_name}/include" with "dummy"
		# Then we will use regex to get the substring after ${_include_match_name}/
		string(REGEX REPLACE "${_include_match_name}[\/\\]include" "dummy" _source_modified "${_source}")
		string(REGEX MATCH "${_include_match_name}[\/\\].*" _source_rel "${_source_modified}")
		#message("${_source_modified} ${_source_rel}")
		if(_source_rel)
			get_filename_component(_source_rel_dir "${_source_rel}" PATH)
			string(REPLACE "/" "\\" _source_group_msvc "${_source_rel_dir}")
			#message("${_source_group_msvc} ${_source}")
			source_group("${_source_group_msvc}" FILES "${_source}")
		endif()
    endforeach()
endfunction(assign_source_group)
