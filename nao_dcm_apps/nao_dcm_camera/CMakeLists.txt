cmake_minimum_required(VERSION 2.8.3)
project(nao_dcm_camera)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  camera_info_manager
  dynamic_reconfigure
  nao_dcm_msgs
  sensor_msgs
)

find_package(NAOqi QUIET COMPONENTS
    alcommon
    alvision
    alproxies
    alerror
    alvalue
)

find_package(Boost)

generate_dynamic_reconfigure_options(cfg/NaoDCMCamera.cfg)

##Needed for ros packages
catkin_package(CATKIN_DEPENDS roscpp std_msgs nao_dcm_msgs sensor_msgs camera_info_manager dynamic_reconfigure)

if( NAOqi_FOUND AND Boost_FOUND)
    message(STATUS "NAOqi C++ SDK found; C++ nodes will be built")
    include_directories( include
        ${catkin_INCLUDE_DIRS}
        ${NAOqi_INCLUDE_DIRS}
    	  ${Boost_INCLUDE_DIRS})
    add_executable(nao_dcm_camera src/nao_camera.cpp)
    target_link_libraries(nao_dcm_camera
        ${catkin_LIBRARIES}
        ${NAOqi_LIBRARIES}
        ${Boost_LIBRARIES})
    add_dependencies(nao_dcm_camera ${catkin_EXPORTED_TARGETS})

else()
    message(STATUS "Cannot find NAOqi C++ sdk; C++ nodes will NOT be built")
endif()

install(TARGETS nao_dcm_camera
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY launch 
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY share
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
