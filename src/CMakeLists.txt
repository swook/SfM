set(
  MAIN_SOURCES
  "util.cpp"
  "0_load_images.cpp"
  "1_feature_extraction.cpp"
  "2_find_matching_pairs.cpp"
  "3_registration.cpp"
  "4_spanning_tree.cpp"
  "5_find_clusters.cpp"
  "6_find_CoM.cpp"
  "7_bundle_adjustment.cpp"
)

add_library(pipeline ${MAIN_SOURCES})

target_link_libraries(pipeline)
