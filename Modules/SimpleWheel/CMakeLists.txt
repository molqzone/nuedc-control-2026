target_sources(xr PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/simple_wheel.cpp")

target_include_directories(xr PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}")
