#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <depth_image_proc/depth_traits.h>
#include <topic_compression/CompressedDepthImage.h>
#include <topic_compression/CompressedImage.h>
#include <topic_compression/EmptyImage.h>
#include "rvl.h"

ros::Publisher publisher;


void not_yet_implemented(const std::string& what) {
    throw ros::Exception("Not Yet Implemented: " + what);
}

ros::master::TopicInfo get_topic_info(std::string topic) {
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::string wanted_topic = topic.erase(topic.find_last_not_of('/') + 1);
    for (auto & master_topic : master_topics) {
        if(master_topic.name == wanted_topic)
            return master_topic;
    }
    throw ros::Exception("Topic '" + topic + "' does not exist.");
}

void image_to_compressed_depth(const sensor_msgs::ImageConstPtr &depth_msg) {
    int U = depth_msg->width;
    int V = depth_msg->height;
    char * output = (char *) malloc(V * U * sizeof(uint16_t));

    auto cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    if ("16UC1" != depth_msg->encoding) {
        ROS_INFO("Encoding is not supported, it must be 16UC1");
    }

    int n = CompressRVL(cv_depth_ptr->image.ptr<uint16_t>(0), output, V * U);

    topic_compression::CompressedDepthImage rvl_msg;

    rvl_msg.header = std_msgs::Header();
    rvl_msg.meta.header = depth_msg->header;
    rvl_msg.meta.height = depth_msg->height;
    rvl_msg.meta.width = depth_msg->width;
    rvl_msg.meta.encoding = depth_msg->encoding;
    rvl_msg.meta.is_bigendian = depth_msg->is_bigendian;
    rvl_msg.meta.step = depth_msg->step;

    rvl_msg.data.data.clear();

    for (int i = 0; i < n; i++) {
        rvl_msg.data.data.push_back(output[i]);
    }

    publisher.publish(rvl_msg);

    free(output);
}


void compressed_depth_to_image(const topic_compression::CompressedDepthImage::ConstPtr &cmp_msg) {
    int U = cmp_msg->meta.width;
    int V = cmp_msg->meta.height;

    auto * output = (uint16_t *) malloc(V * U * sizeof(uint16_t));
    char* dataMat = (char *) malloc(V * U * sizeof(uint16_t));

    int i = 0;
    for (signed char it : cmp_msg->data.data) {
        dataMat[i] = it;
        i++;
    }

    DecompressRVL((char *) dataMat, (uint16_t *) output, V * U);
    cv::Mat image(V, U, CV_16UC1, output);

    cv_bridge::CvImagePtr cv_depth_ptr(new cv_bridge::CvImage);

    cv_depth_ptr->encoding = "16UC1";
    cv_depth_ptr->header = cmp_msg->meta.header;
    cv_depth_ptr->image = image;

    publisher.publish(cv_depth_ptr->toImageMsg());

    free(output);
    free(dataMat);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "topic_compression", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    // Get in topic and out topic names
    std::string in_topic = nh.resolveName("in");
    std::string out_topic = nh.resolveName("out");
    if (in_topic.empty() || in_topic == "/in") {
        printf("Usage: %s in:=<in_base_topic> out:=<out_base_topic>\n", argv[0]);
        return 0;
    }
    if (in_topic[in_topic.length()] != '/')
        in_topic += '/';

    std::cout << "IN: " << in_topic << std::endl;
    std::cout << "OUT: " << out_topic << std::endl;

    ros::master::TopicInfo info = get_topic_info(in_topic);
    // TODO: Format this similar to image_transport with clear in and out transports for pub/subs
    // TODO: This is horrible make this dynamic
    if(info.datatype == "sensor_msgs/Image") {
        if (out_topic.empty() || out_topic == "/out")
            out_topic = in_topic + "compressed";
        auto msg_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(in_topic);
        if(msg_ptr->encoding == "16UC1") {
            ROS_INFO_STREAM("Compressing depth from '" << in_topic << "' to '" << out_topic << "'");
            publisher = nh.advertise<topic_compression::CompressedDepthImage>(out_topic, 30);
            image_transport::ImageTransport it(nh);
            image_transport::Subscriber subscriber = it.subscribe(in_topic, 1, image_to_compressed_depth);
            ros::spin();
        } else if (std::string("rgb8bgr88UC3").find(msg_ptr->encoding) != std::string::npos) {
            ROS_INFO_STREAM("Compressing colour from '" << in_topic << "' to '" << out_topic << "'");
            not_yet_implemented("Colour Compression");
        } else {
            ROS_ERROR_STREAM("Compressing encoding '" << msg_ptr->encoding << "' not currently supported.");
            return 0;
        }
    } else if (info.datatype == "topic_compression/CompressedImage") {
        if (out_topic.empty() || out_topic == "/out")
            out_topic = in_topic + "decompressed";
        ROS_INFO_STREAM("Decompressing colour from '" << in_topic << "' to '" << out_topic << "'");
        not_yet_implemented("Colour Decompression");
    } else if (info.datatype == "topic_compression/CompressedDepthImage") {
        if (out_topic.empty() || out_topic == "/out")
            out_topic = in_topic + "decompressed";
        ROS_INFO_STREAM("Decompressing depth from '" << in_topic << "' to '" << out_topic << "'");
        publisher = nh.advertise<sensor_msgs::Image>(out_topic, 30);
        ros::Subscriber subscriber = nh.subscribe<topic_compression::CompressedDepthImage>(in_topic, 1, compressed_depth_to_image);
        ros::spin();
    } else {
        ROS_ERROR_STREAM("No valid compression/decompression for message of type '" << info.datatype << "'");
        return -1;
    }
//    ros::Subscriber subscriber = nh.subscribe<>()
//    publisher = nh.advertise<topic_compression::CompressedDepthImage>(out_topic, 30);
    return 0;
}



