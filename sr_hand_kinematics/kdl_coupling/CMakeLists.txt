cmake_minimum_required(VERSION 2.8.3)
project(kdl_coupling)
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(orocos_kdl REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

catkin_package(
        DEPENDS orocos_kdl
        CATKIN_DEPENDS roscpp
        INCLUDE_DIRS include
        LIBRARIES kdl_coupling
)

#add_definitions(-DEIGEN2_SUPPORT)
add_library(kdl_coupling src/chain_coupling.cpp src/chainjnttojacsolver_coupling.cpp src/chainiksolvervel_wdls_coupling.cpp src/chainiksolverpos_nr_jl_coupling.cpp)
add_dependencies(kdl_coupling ${catkin_EXPORTED_TARGETS})
target_link_libraries(kdl_coupling ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS kdl_coupling
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
