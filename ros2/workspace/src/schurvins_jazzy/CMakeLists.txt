cmake_minimum_required(VERSION 3.22)
project(schurvins_jazzy LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_library(OPENGV_LIBRARY opengv REQUIRED)
get_filename_component(WS "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(UP "${WS}/upstream/SchurVINS")
set(GEN "${CMAKE_CURRENT_BINARY_DIR}/generated")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py")
execute_process(COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/prepare_sources.py"
  "${UP}" "${GEN}" COMMAND_ERROR_IS_FATAL ANY)

set(CORE_SOURCES)
foreach(module svo svo_common svo_direct svo_tracker)
  file(GLOB sources CONFIGURE_DEPENDS "${UP}/${module}/src/*.cpp")
  list(APPEND CORE_SOURCES ${sources})
endforeach()
list(REMOVE_ITEM CORE_SOURCES "${UP}/svo_direct/src/feature_detection_utils.cpp")
list(APPEND CORE_SOURCES "${GEN}/feature_detection_utils.cpp")
foreach(name homography math_utils performance_monitor sample user_input_thread vision)
  list(APPEND CORE_SOURCES "${UP}/vikit/vikit_common/src/${name}.cpp")
endforeach()
file(GLOB cameras CONFIGURE_DEPENDS "${UP}/vikit/vikit_cameras/src/*.cpp")
list(APPEND CORE_SOURCES ${cameras}
  "${UP}/vikit/vikit_solver/src/robust_cost.cpp"
  "${UP}/rpg_common/src/fs.cpp"
  "${UP}/svo_img_align/src/sparse_img_align.cpp"
  "${UP}/svo_img_align/src/sparse_img_align_base.cpp"
  "${GEN}/svo_factory.cpp")
set(FAST "${WS}/upstream/fast_neon")
foreach(name fast_9 fast_10 fast_10_score nonmax_3x3)
  list(APPEND CORE_SOURCES "${FAST}/src/${name}.cpp")
endforeach()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
  list(APPEND CORE_SOURCES "${FAST}/src/faster_corner_10_sse.cpp")
endif()
add_library(schurvins_core SHARED ${CORE_SOURCES})
target_compile_definitions(schurvins_core PUBLIC SVO_USE_OPENGV SVO_DEPTHFILTER_IN_REPROJECTOR)
target_compile_options(schurvins_core PRIVATE -Wno-deprecated-declarations
  -include "${CMAKE_CURRENT_SOURCE_DIR}/include/hexbot_vio/compat.hpp")
target_include_directories(schurvins_core PUBLIC
  "${GEN}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${WS}/upstream/minkindr/minkindr/include"
  "${WS}/upstream/eigen_checks/include" "${FAST}/include"
  "${UP}/svo/include" "${UP}/svo_common/include" "${UP}/svo_direct/include"
  "${UP}/svo_img_align/include" "${UP}/svo_tracker/include"
  "${UP}/svo_vio_common/include" "${UP}/rpg_common/include"
  "${UP}/vikit/vikit_common/include" "${UP}/vikit/vikit_cameras/include"
  "${UP}/vikit/vikit_solver/include" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(schurvins_core PUBLIC Eigen3::Eigen yaml-cpp glog::glog
  gflags ${OpenCV_LIBS} ${OPENGV_LIBRARY} Boost::filesystem Boost::system Boost::thread)
add_executable(stereo_vio src/stereo_vio.cpp src/tag_initialization.cpp)
target_link_libraries(stereo_vio schurvins_core)
ament_target_dependencies(stereo_vio rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge message_filters tf2_ros)
install(TARGETS schurvins_core stereo_vio
  LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_export_libraries(schurvins_core)
ament_package()
