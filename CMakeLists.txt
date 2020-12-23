cmake_minimum_required(VERSION 3.0.2)
project(create_navigation)

add_compile_options(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    create_msgs
    pcl_ros
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        create_msgs
        pcl_ros
)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
)

add_executable(create_pointcloud_node
    src/create_pointcloud_node.cpp
)
target_link_libraries(create_pointcloud_node
    ${catkin_LIBRARIES}
)


#-----------------------------------------------------------------------------------------------------------------------
# Install
#-----------------------------------------------------------------------------------------------------------------------

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_create_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
