add_executable(wbc_g1_ctrl wbc_g1_ctrl/main.cpp)
set_target_properties(wbc_g1_ctrl PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_executable(wbc_ref_pub wbc_ref_pub/main.cpp)
set_target_properties(wbc_ref_pub PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_executable(wbc_ref_listen wbc_ref_listen/main.cpp)
set_target_properties(wbc_ref_listen PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

add_executable(wbc_reference_node wbc_reference_node/main.cpp)
set_target_properties(wbc_reference_node PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
