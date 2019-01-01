///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////

#include "duo_manager.h"

// constructor
DuoManager::DuoManager():nh("~"),it(nh),frame_recieved(false) {

    // parameter parsing
    nh.param<std::string>("world_frame_id",world_frame_id,"world");
    nh.param<std::string>("camera_frame_id",camera_frame_id,"camera_frame");

    nh.param("loop_rate",params.rate,30.0f);

    nh.param("max_sensing_depth",max_depth,3.0f);
    nh.param("min_sensing_depth",min_depth,0.1f);

    nh.param<std::string>("pcl_topic",pcl_topic_id,"point_cloud");
    nh.param<std::string>("img_topic",img_topic_id,"raw_rgb");
    nh.param<std::string>("img_topic",depth_topic_id,"raw_depth");

    nh.param("depth_mode",params.depth_mode,2);
    nh.param("width",params.width,320);
    nh.param("height",params.height,240);

    // publishers
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2>(pcl_topic_id,1);
    pub_img_raw = it.advertise(img_topic_id,1);
    pub_img_depth = it.advertise(depth_topic_id,1);

    // data init
    colorLut = Mat(cv::Size(256, 1), CV_8UC3);
    for(int i = 0; i < 256; i++)
        colorLut.at<Vec3b>(i) = (i==0) ? Vec3b(0, 0, 0) : HSV2RGB(i/256.0f, 1, 1);
    point_cloud.width = params.width;
    point_cloud.height = params.height;

    img_raw = Mat(Size(params.width, params.height), CV_32FC3);
    disparity = Mat(Size(params.width, params.height), CV_32FC1);
    depth3d = Mat(Size(params.width, params.height), CV_32FC3);



};

// destructor
DuoManager::~DuoManager() {
    Dense3DClose(dense3d);
    CloseDUOCamera();
}

int DuoManager::init() {
    // this proceed required parameter initialization for duo
    // returns

    // Open DUO camera and start capturing
    if(!OpenDUOCamera(params.width, params.height, params.rate))
    {
        printf("Could not open DUO camera\n");
        return 0;
    }

    printf("DUOLib Version:       v%s\n", GetDUOLibVersion());
    printf("Dense3D Version:      v%s\n", Dense3DGetLibVersion());

    // Open Dense3D
    if(!Dense3DOpen(&dense3d))
    {
        printf("Could not open Dense3D library\n");
        // Close DUO camera
        CloseDUOCamera();
        return 1;
    }
    // Set the Dense3D license (visit https://duo3d.com/account)
    if(!SetDense3DLicense(dense3d, "33ABM-6HLC5-ARFNL-HCE4B-NM4EV")) // <-- Put your Dense3D license
    {
        printf("Invalid or missing Dense3D license. To get your license visit https://duo3d.com/account\n");
        // Close DUO camera
        CloseDUOCamera();
        // Close Dense3D library
        Dense3DClose(dense3d);
        return 0;
    }
    // Set the image size
    if(!SetDense3DImageSize(dense3d, (uint32_t)params.width, (uint32_t)params.height))
    {
        printf("Invalid image size\n");
        // Close DUO camera
        CloseDUOCamera();
        // Close Dense3D library
        Dense3DClose(dense3d);
        return 0;
    }
    // Get DUO calibration intrinsics and extrinsics
    DUO_STEREO param;
    if(!GetCameraStereoParameters(&param))
    {
        printf("Could not get DUO camera calibration data\n");
        // Close DUO camera
        CloseDUOCamera();
        // Close Dense3D library
        Dense3DClose(dense3d);
        return 0;
    }

    // Set Dense3D parameters
    SetDense3DScale(dense3d, 0);
    SetDense3DMode(dense3d, (uint32_t)params.depth_mode);
    SetDense3DCalibration(dense3d, &param);
    SetDense3DNumDisparities(dense3d, 4);
    SetDense3DSADWindowSize(dense3d, 6);
    SetDense3DPreFilterCap(dense3d, 50);
    SetDense3DUniquenessRatio(dense3d, 27);
    SetDense3DSpeckleWindowSize(dense3d, 52);
    SetDense3DSpeckleRange(dense3d, 14);
    // Set exposure, LED brightness and camera orientation
    SetExposure(85);
    SetLed(50);
    SetVFlip(false);
    // Enable retrieval of undistorted (rectified) frames
    SetUndistort(true);
    return 1; // successful for config

}

bool DuoManager::frame_update() {
    PDUOFrame pFrameData = GetDUOFrame();

    if(pFrameData == NULL) return false;

    // Create Mat for left & right frames
    Mat left = Mat(Size(params.width, params.height), CV_8UC1, pFrameData->leftData);
    img_raw = left;
//    Mat right = Mat(Size(params.width, params.height), CV_8UC1, pFrameData->rightData);

//    img_raw = left;

    // Process Dense3D depth map here
    if(Dense3DGetDepth(dense3d, pFrameData->leftData, pFrameData->rightData,
                       (float*)disparity.data, (PDense3DDepth)depth3d.data))
    {
        // 1. depth map update
        uint32_t disparities; // here, Disparity is just scaling factor for representation
        GetDense3DNumDisparities(dense3d, &disparities);
        Mat disp8;
        disparity.convertTo(disp8, CV_8UC1, 255.0/(disparities*16));
        cvtColor(disp8, img_depth, COLOR_GRAY2BGR);
        LUT(img_depth, colorLut, img_depth); // colormap change

//
//        imshow("Dense3D Disparity Map", img_depth);
//        imshow("img_rgb", img_raw);
//
//        cvWaitKey(1);


        // 2. point cloud update
        uint8_t *color = left.data;
        PDense3DDepth depth = (PDense3DDepth)depth3d.data;
        point_cloud.clear();
        for(int j = 0; j < left.total(); j++)
        {
            if((depth[j].z >= max_depth * 1000.0f) or (depth[j].z <= min_depth * 1000.0f) ) continue;
            pcl::PointXYZRGB p;
            p.x = depth[j].x * 0.001f;
            p.y = depth[j].y * 0.001f;
            p.z = depth[j].z * 0.001f;
            uint32_t rgb = ((uint32_t)color[j] << 16 | (uint32_t)color[j] << 8 | (uint32_t)color[j]);
            p.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud.push_back(p);
        }

        frame_recieved = true;
        return true;
    }
}

void DuoManager::publish() {

    header.frame_id = camera_frame_id;
    header.stamp = ros::Time::now();

    if (pub_img_raw.getNumSubscribers()>0 & frame_recieved)
        pub_img_raw.publish(imageToROSmsg(img_raw,
                sensor_msgs::image_encodings::MONO8, camera_frame_id, ros::Time::now()));
    if (pub_img_depth.getNumSubscribers()>0 & frame_recieved)
        pub_img_depth.publish(imageToROSmsg(img_depth,
                sensor_msgs::image_encodings::BGR8, camera_frame_id, ros::Time::now()));
    if (pub_pcl.getNumSubscribers()>0 & frame_recieved) {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(point_cloud, output);
        output.header = header;
        pub_pcl.publish(output);
    }
}


int main(int argc, char* argv[])
{
    ros::init(argc,argv,"duo_node");
    DuoManager duo_manager;
    int init = duo_manager.init();

    while (ros::ok() & init)
    {
        duo_manager.frame_update();
        duo_manager.publish();
        ros::spinOnce();
    }


    return 0;
}

