cmake_minimum_required(VERSION 3.8)
project(fastrack_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(fastrack_core_lib
  src/t5D_p3D_dublins_car_dynamics.cpp
)

target_include_directories(fastrack_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(fastrack_core_lib
  Eigen3::Eigen
)
ament_target_dependencies(fastrack_core_lib
  rclcpp Eigen3
)


function(add_dynamic_test test_name test_source)
  ament_add_gtest(${PROJECT_NAME}_${test_name} ${test_source})
  target_include_directories(${PROJECT_NAME}_${test_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
  )
  target_link_libraries(${PROJECT_NAME}_${test_name}
    Eigen3::Eigen
    fastrack_core_lib
  )

endfunction()


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_dynamic_test(t5D_p3D_dublins_car_dynamics test/test_t5D_p3D_dublins_car_dynamics.cpp)

endif()

# target_link_libraries(Eigen3::Eigen)
ament_package()
