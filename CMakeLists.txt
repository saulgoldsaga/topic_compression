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
        image_transport
        depth_image_proc
        cv_bridge
        message_generation
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
catkin_package(CATKIN_DEPENDS std_msgs message_runtime cv_bridge)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include/
)

add_executable(run src/bidirectional.cpp include/rvl.h)
target_link_libraries(run ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})