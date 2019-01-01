//
// Created by jbs on 18. 12. 31.
//

#ifndef DUO3D_SELF_DUO_MANAGER_H
#define DUO3D_SELF_DUO_MANAGER_H

#include "utils.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>


struct DuoParams{
    int width,height; //[320 240] recommended
    float rate;
    int depth_mode; // 0-3

};


class DuoManager{
public:
    // frame and topic
    std::string camera_frame_id;
    std::string world_frame_id;
    std::string pcl_topic_id;
    std::string img_topic_id;
    std::string depth_topic_id;

    // msg header
    std_msgs::Header header;

    // node
    ros::NodeHandle nh;
    ros::Publisher pub_pcl;
    image_transport::ImageTransport it;
    image_transport::Publisher pub_img_raw;
    image_transport::Publisher pub_img_depth;


    // data
    Mat colorLut;
    Mat img_raw; // rgb (gray scale)
    Mat img_depth; //colored depth
    Mat1f disparity;
    Mat3f depth3d;
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud;


    // duo
    Dense3DInstance  dense3d;
    float max_depth;
    float min_depth;
    DuoParams params;


    // methods
    DuoManager(); // contructor
    int init(); // initialize the parameter settings
    bool frame_update(); // get frame and construct pcl for duo
    void publish(); // publish required data
    ~DuoManager(); //destructor

    // flag
    bool frame_recieved;

};



#endif //DUO3D_SELF_DUO_MANAGER_H
