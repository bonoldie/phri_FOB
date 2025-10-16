cmake_minimum_required(VERSION 2.8.3)
project(netft_rdt_driver)

find_package(catkin REQUIRED COMPONENTS roscpp diagnostic_updater diagnostic_msgs message_generation)

find_package(
  Boost REQUIRED COMPONENTS system thread program_options
)

#add_definitions(-DBOOST_LOG_DYN_LINK=1)


add_service_files(DIRECTORY srv FILES
  Zero.srv
)

generate_messages()

catkin_package()

message("Boost_LIBRARIES " ${Boost_LIBRARIES})
message("Boost_INCLUDE_DIRS " ${Boost_INCLUDE_DIRS})
include_directories( include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Driver library that can be used by other stuff (ie : netft_ethercat_hardware) 
add_library(${PROJECT_NAME} src/netft_rdt_driver.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ) 
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

# Stand-alone ROS node that uses driver library 
add_executable(netft_node src/netft_node.cpp)
target_link_libraries(netft_node ${PROJECT_NAME}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(netft_node ${PROJECT_NAME})
