idf_component_register(INCLUDE_DIRS ".")
target_link_libraries(${COMPONENT_LIB} INTERFACE "-L ${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(${COMPONENT_LIB} INTERFACE algobsec)