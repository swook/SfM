set(
  MAIN_SOURCES
  "util.cpp"
  "Associativity.cpp"
  "0_load_images.cpp"
  "1_feature_extraction.cpp"
  "2_find_matching_pairs.cpp"
  "3_registration.cpp"
  "4_spanning_tree.cpp"
  "5_global_cam_poses.cpp"
  "6_find_clusters.cpp"
  "7_find_CoM.cpp"
  "8_bundle_adjustment.cpp"
)

add_library(pipeline ${MAIN_SOURCES})

target_link_libraries(pipeline)
