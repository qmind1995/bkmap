set(FOLDER_NAME "exe")

BKMAP_ADD_EXECUTABLE(automatic_reconstructor automatic_reconstructor.cpp)

BKMAP_ADD_EXECUTABLE(bundle_adjuster bundle_adjuster.cpp)
#
BKMAP_ADD_EXECUTABLE(bkmap bkmap.cpp)
#
BKMAP_ADD_EXECUTABLE(color_extractor color_extractor.cpp)

BKMAP_ADD_EXECUTABLE(database_creator database_creator.cpp)

BKMAP_ADD_EXECUTABLE(dense_fuser dense_fuser.cpp)

BKMAP_ADD_EXECUTABLE(dense_mesher dense_mesher.cpp)

BKMAP_ADD_EXECUTABLE(exhaustive_matcher exhaustive_matcher.cpp)

BKMAP_ADD_EXECUTABLE(feature_extractor feature_extractor.cpp)

BKMAP_ADD_EXECUTABLE(feature_importer feature_importer.cpp)

BKMAP_ADD_EXECUTABLE(image_rectifier image_rectifier.cpp)

BKMAP_ADD_EXECUTABLE(image_registrator image_registrator.cpp)

BKMAP_ADD_EXECUTABLE(image_undistorter image_undistorter.cpp)

BKMAP_ADD_EXECUTABLE(mapper mapper.cpp)

BKMAP_ADD_EXECUTABLE(matches_importer matches_importer.cpp)

BKMAP_ADD_EXECUTABLE(model_aligner model_aligner.cpp)

BKMAP_ADD_EXECUTABLE(model_analyzer model_analyzer.cpp)

BKMAP_ADD_EXECUTABLE(model_converter model_converter.cpp)

BKMAP_ADD_EXECUTABLE(model_merger model_merger.cpp)

BKMAP_ADD_EXECUTABLE(model_orientation_aligner model_orientation_aligner.cpp)

BKMAP_ADD_EXECUTABLE(point_triangulator point_triangulator.cpp)

BKMAP_ADD_EXECUTABLE(rig_bundle_adjuster rig_bundle_adjuster.cpp)

BKMAP_ADD_EXECUTABLE(sequential_matcher sequential_matcher)

BKMAP_ADD_EXECUTABLE(spatial_matcher spatial_matcher.cpp)

BKMAP_ADD_EXECUTABLE(transitive_matcher transitive_matcher.cpp)

BKMAP_ADD_EXECUTABLE(vocab_tree_builder vocab_tree_builder.cpp)

BKMAP_ADD_EXECUTABLE(vocab_tree_matcher vocab_tree_matcher.cpp)

BKMAP_ADD_EXECUTABLE(vocab_tree_retriever vocab_tree_retriever.cpp)

#if(CUDA_ENABLED)
#    BKMAP_ADD_EXECUTABLE(dense_stereo dense_stereo.cc)
#    target_include_directories(dense_stereo PUBLIC ${CUDA_INCLUDE_DIRS})
#endif()
