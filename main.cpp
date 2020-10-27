//#include <QCoreApplication>
#include <iostream>

#include "include/cloud_pro.h"
//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/PointCloud2.h>

//int g_client_fd=0;

/*
//void callback_pointcloud (const sensor_msgs::PointCloud2ConstPtr& input)
void callback_pointcloud (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
 // 创建一个输出的数据格式
 // sensor_msgs::PointCloud2 output;  //ROS中点云的数据格式
  //output = *input;

//  //对数据进行处理
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//  pcl::fromROSMsg(output,*cloud);
//     //blocks until the cloud is actually rendered
//  viewer.showCloud(cloud);

  //pub.publish (output);
    std::cout<<" cloud point_step: "<< cloud_msg->point_step << " ..........." <<std::endl;
  std::cout<<" wishchin counter: "<< g_client_fd<< " ..........." <<std::endl;
  g_client_fd +=1;
}
*/


int main(int argc, char *argv[])
{
    //QCoreApplication a(argc, argv);
    //std::cout<< "helllo,wishchin! "<<std::endl;
    //cloud_pro(argc,argv);
    pcd_icp(argc,argv);

/*
        ros::init(argc, argv, "rtabmap");
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe("/rtabmap/cloud_map", 1, callback_pointcloud);

        ros::Rate loop_rate(1.0);
        while (ros::ok()){
            std::cout<<" innner counter: "<< g_client_fd<< " ..........." <<std::endl;
            ros::spinOnce();
            bool met = loop_rate.sleep();
        }

        //创建ROS的发布节点
        //pub = nh.advertise<sensor_msgs::PointCloud2> (“output”, 1);
        printf("call sender over!\n");
        //ros::spin();

    //    // 关闭socket文件
    //    close(sockfd);
    //    close(client_fd);
*/

    //return a.exec();
}
