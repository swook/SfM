set(
  MAIN_SOURCES
  "0_load_images.cpp"
  "1_feature_extraction.cpp"
  "2_feature_match.cpp"
  "3_pose_estimates.cpp"
  "4_registration.cpp"
  "5_to_global_coords.cpp"
  "6_bundle_adjustment.cpp"
)

add_library(pipeline ${MAIN_SOURCES})

target_link_libraries(pipeline)
