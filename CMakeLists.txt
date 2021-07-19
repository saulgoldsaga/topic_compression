cmake_minimum_required(VERSION 2.8.3)
project(topic_compression)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# ==============================================
# Catkin setup
# ==============================================
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        genmsg
        std_msgs
        sensor_msgs
        message_generation
        cv_bridge
        image_transport
        pybind11_catkin
)

# ==============================================
# Python files setup
# ==============================================
#catkin_python_setup()


# ==============================================
# Service/Message files setup
# ==============================================

add_message_files(
        FILES
        CompressedDepthImage.msg
        CompressedImage.msg
        CompressionMeta.msg
)


generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)


# ==============================================
# Setup
# ==============================================
catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs message_runtime cv_bridge pybind11_catkin)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include/
)

pybind_add_module(topic_compression_lib MODULE src/python_wrapper.cpp)

set(HEADER_FILES include/rvl.hpp)
add_executable(run src/bidirectional.cpp ${HEADER_FILES})
add_dependencies(run topic_compression_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(run ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
