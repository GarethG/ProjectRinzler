FILE(REMOVE_RECURSE
  "../src/beginner_tutorials/msg"
  "../src/beginner_tutorials/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/add_two_ints_server.dir/src/add_two_ints_server.o"
  "../bin/add_two_ints_server.pdb"
  "../bin/add_two_ints_server"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/add_two_ints_server.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
