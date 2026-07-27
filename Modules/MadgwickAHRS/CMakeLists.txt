target_sources(xr PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/MadgwickAHRS.cpp")

target_include_directories(xr PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}")
