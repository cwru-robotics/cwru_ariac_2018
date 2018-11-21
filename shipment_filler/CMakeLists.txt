cmake_minimum_required(VERSION 2.8.3)
project(shipment_filler)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

if(MSVC)  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zi")  
else()  
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")  
endif()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(ShipmentFiller src/ShipmentFiller.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
#cs_add_executable(shipment_filler src/shipment_filler.cpp)
#cs_add_executable(shipment_filler_test_bad_part src/shipment_filler_test_bad_part.cpp)
cs_add_executable(simple_shipment_filler src/simple_shipment_filler.cpp)
cs_add_executable(test_pick_from_inventory src/test_pick_from_inventory.cpp)
cs_add_executable(test_part_placement_from_inventory src/test_part_placement_from_inventory.cpp)
cs_add_executable(test_robot_behaviors src/test_robot_behaviors.cpp)
cs_add_executable(test_pick_from_box src/test_pick_from_box.cpp)
#cs_add_executable(simple_shipment_filler_wsn_test src/simple_shipment_filler_wsn_test.cpp)
cs_add_executable(simple_shipment_filler_w_optimizer src/simple_shipment_filler_w_optimizer.cpp)
cs_add_executable(shipment_filler_test src/shipment_filler_test.cpp)
cs_add_executable(unload_box src/unload_box.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
#target_link_libraries(shipment_filler ShipmentFiller)
#target_link_libraries(shipment_filler_test_bad_part ShipmentFiller)
target_link_libraries(test_pick_from_box ShipmentFiller) 
target_link_libraries(simple_shipment_filler ShipmentFiller) 
#target_link_libraries(simple_shipment_filler_wsn_test ShipmentFiller) 
target_link_libraries(simple_shipment_filler_w_optimizer ShipmentFiller) 
#target_link_libraries(simple_shipment_filler_wsn_test ShipmentFiller) 
target_link_libraries(shipment_filler_test ShipmentFiller)

cs_install()
cs_export()
    
