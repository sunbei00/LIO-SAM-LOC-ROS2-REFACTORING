cmake_minimum_required(VERSION 3.5)
project(lio_sam_loc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set policies
if (POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if (POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if (POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC OFF)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(geometry_msgs REQUIRED)

if (OpenMP_CXX_FOUND)
  set(USE_OPENMP OpenMP::OpenMP_CXX)
endif ()

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include/lio_sam
)

set(COMMON_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/QoS.cpp")
set(PCL_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/pclUtils.cpp")
set(GTSAM_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/gtsamUtils.cpp")

set(COMMON_DEP1 rclcpp rclpy  tf2_eigen tf2_geometry_msgs pcl_conversions)
set(COMMON_DEP2 ${COMMON_DEP1} nav_msgs sensor_msgs std_msgs)


# featureExtraction
add_executable(${PROJECT_NAME}_featureExtraction
        src/nodes/nodeFE.cpp
        src/components/FeatureExtraction.cpp
        ${COMMON_SRC}
        ${PCL_SRC})
ament_target_dependencies(${PROJECT_NAME}_featureExtraction
        ${COMMON_DEP1}
        PCL)
target_link_libraries(${PROJECT_NAME}_featureExtraction
        "${cpp_typesupport_target}")


# imageProjection
add_executable(${PROJECT_NAME}_imageProjection
        src/nodes/nodeIP.cpp
        src/components/ImageProjection.cpp
        ${COMMON_SRC}
        ${PCL_SRC})
ament_target_dependencies(${PROJECT_NAME}_imageProjection
        ${COMMON_DEP2}
        PCL)
target_link_libraries(${PROJECT_NAME}_imageProjection
        "${cpp_typesupport_target}")


# imuPreintegration
add_executable(${PROJECT_NAME}_imuPreintegration
        src/nodes/nodeIPI.cpp
        src/components/ImuPreintegration.cpp
        src/components/TransformFusion.cpp
        ${COMMON_SRC})
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration
        ${COMMON_DEP2}
        GTSAM)
target_link_libraries(${PROJECT_NAME}_imuPreintegration
        gtsam
        "${cpp_typesupport_target}")


# mapOptimization
add_executable(${PROJECT_NAME}_mapOptimization
        src/nodes/nodeMO.cpp
        src/components/MapOptmization.cpp
        src/components/MOInitializer.cpp
        src/components/MOVisualize.cpp
        src/components/MOPublish.cpp
        src/components/MOGlobalMatching.cpp
        src/components/MOLocalization.cpp
        ${COMMON_SRC}
        ${PCL_SRC}
        ${GTSAM_SRC})
ament_target_dependencies(${PROJECT_NAME}_mapOptimization
        ${COMMON_DEP2}
        PCL
        visualization_msgs
        tf2
        OpenCV
        GTSAM
        geometry_msgs)
target_link_libraries(${PROJECT_NAME}_mapOptimization
        gtsam
        "${cpp_typesupport_target}"
        ${USE_OPENMP})


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}_imageProjection
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_imuPreintegration
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_featureExtraction
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_mapOptimization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

