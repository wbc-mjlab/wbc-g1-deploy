add_executable(wbc_g1_ctrl wbc_g1_ctrl/main.cpp)
set_target_properties(wbc_g1_ctrl PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
