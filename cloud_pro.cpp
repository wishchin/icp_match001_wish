//#include <QCoreApplication>
#include <iostream>
#include <cmath>
#include <ctime>
#include <sys/time.h>
#include "./include/cloud_pro.h"

//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/PointCloud2.h>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl_conversions/pcl_conversions.h>
//三角化
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
//#include <boost/thread/thread.hpp>

//点云滤波
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>//统计滤波 头文件
#include <pcl/filters/voxel_grid.h>//体素滤波 头文件
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>

//#include <pcl/correspondence.h>   //对应表示两个实体之间的匹配（例如，点，描述符等）。
#include <pcl/features/normal_3d_omp.h>   //法线
//#include <pcl/features/shot_omp.h>    //描述子
//#include <pcl/features/board.h>
//#include <pcl/keypoints/uniform_sampling.h>   //均匀采样
//#include <pcl/recognition/cg/hough_3d.h>    //hough算子
//#include <pcl/recognition/cg/geometric_consistency.h>  //几何一致性
#include <pcl/kdtree/kdtree_flann.h>             //配准方法
#include <pcl/kdtree/impl/kdtree_flann.hpp>      //
//#include <pcl/common/transforms.h>             //转换矩阵
//#include <pcl/console/parse.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define SHOW_CLOUD

//定义点云法法线相关结构
typedef pcl::PointXYZRGB PointType;  //PointXYZRGBA数据结构
typedef pcl::Normal NormalType;       //法线结构
typedef pcl::ReferenceFrame RFType;    //参考帧
typedef pcl::SHOT352 DescriptorType;   //SHOT特征的数据结构http://www.cnblogs.com/li-yao7758258/p/6612856.html

// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
#ifdef SHOW_CLOUD
    static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("cloud Viewer1"));
    static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_tmp (new pcl::visualization::PCLVisualizer ("cloud ViewerT"));
    static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_src (new pcl::visualization::PCLVisualizer ("cloud Viewer_src"));
    static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_dst (new pcl::visualization::PCLVisualizer ("cloud Viewer_dst"));
#endif

int g_client_fd=0;
float g_color_fill = 10000;
float g_thick = 0.6;//地板和天花板厚度
float g_ceil_height = 1.3;
float g_floor_height = -0.5;
bool g_skip_nans= false;
float g_RadiusSearch =0.3;
bool is_debug = true;

void show_cloud_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer)
//void show_cloud_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
      viewer->setBackgroundColor (0, 0, 0);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
      viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      //viewer->spin();

    return;
}

void show_cloud_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2,boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer)
//void show_cloud_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
      viewer->setBackgroundColor (0, 0, 0);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
      viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud2);
      viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, rgb2, "sample cloud2");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      //viewer->spin();

    return;
}

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}



class MathCal
{
public:
    MathCal(){};
    ~MathCal(){};


public:
    static inline void cutValue(int &inv, const int start, const int end)
    {
        if (inv < start) inv = start;
        if (inv > end) inv = end;
        //return inv;
    }


    //获取包围盒的最大最小值//返回包围盒
    //注意：获取包围盒，需要放大100倍，intelRealSense中1表示1米
    static std::vector<float> findMinMax(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,std::vector<std::pair<float,float> > &minmax, float scale)
    {

        float xmin = 1000000.0;
        float xmax = -1000000.0;
        float ymin = 1000000.0;
        float ymax = -1000000.0;
        float zmin = 1000000.0;
        float zmax = -1000000.0;

        for(int i=0;i< cloud->points.size();++i)
        {
            pcl::PointXYZRGB p = cloud->points[i];
            if( xmin >p.x ) xmin =p.x;
            if( xmax <p.x ) xmax =p.x;
            if( ymin >p.y ) ymin =p.y;
            if( ymax <p.y ) ymax =p.y;
            if( zmin >p.z ) zmin =p.z;
            if( zmax <p.z ) zmax =p.z;
        }

        minmax.push_back(std::make_pair(xmin,xmax) );
        minmax.push_back(std::make_pair(ymin,ymax) );
        minmax.push_back(std::make_pair(zmin,zmax) );

        int bx, by, bz;
        bx = ceil(scale*(minmax[0].second - minmax[0].first));
        by = ceil(scale*(minmax[1].second - minmax[1].first));
        bz = ceil(scale*(minmax[2].second - minmax[2].first));
        std::vector<float> bbx;
        bbx.push_back(bx); bbx.push_back(by); bbx.push_back(bz);

        return bbx;
    }

};

class CViewExtract
{

public:
    CViewExtract(){};
    ~CViewExtract(){};


public:

    //解码RGB值
    Eigen::Vector4f decodeRGB(int rgb)
    {
        Eigen::Vector4f rgbv;

        return rgbv;
    }

    float viewTrans(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloudSrc,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloudTrans,
        Eigen::Vector4f  &AngleTrans)
    {
        //1. Trans the VIew...
        float AngleA, AngleB, AngleC;//声明视角//初始化 作为原始角度
        AngleA = AngleTrans[0];//49.0/pi;
        AngleB = AngleTrans[1];//78.9/pi;
        AngleC = AngleTrans[2];//34.8/pi;

        int size = cloudSrc->points.size();
        cloudTrans->resize(0);
        cloudTrans->reserve(size);

        //位姿识别角度变换矩阵/
        Eigen::Matrix4f TransX, TransY, TransZ;
        //初始化三个矩阵！变换！
        TransX << 1, 0, 0, 0,
            0, cos(AngleA), -sin(AngleA), 0,
            0, sin(AngleA), cos(AngleA), 0,
            0, 0, 0, 1;//

        TransY << cos(AngleB), 0, sin(AngleB), 0,
            0, 1, 0, 0,
            -sin(AngleB), 0, cos(AngleB), 0,
            0, 0, 0, 1;

        TransZ << cos(AngleC), -sin(AngleC), 0, 0,
            sin(AngleC), cos(AngleC), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

        //点云模型角度变换
        Eigen::Vector4f Centroid;
        Centroid << 0, 0, 0, 0;
        pcl::compute3DCentroid(*cloudSrc, Centroid);
        for (int idx = 0; idx < cloudSrc->points.size(); ++idx){

            Eigen::Vector4f PointSrc, PointDest;//维数一致！
            PointSrc[0] = cloudSrc->points[idx].x - Centroid[0];
            PointSrc[1] = cloudSrc->points[idx].y - Centroid[1];
            PointSrc[2] = cloudSrc->points[idx].z - Centroid[2];
            //PointSrc[3] = 1;
            PointDest = (TransX*(TransY*(TransZ*PointSrc)));//创建矩阵无效！

            //cloudSrc->points[idx].x = PointDest[0] + Centroid[0];
            //cloudSrc->points[idx].y = PointDest[1] + Centroid[1];
            //cloudSrc->points[idx].z = PointDest[2] + Centroid[2];
            //cloudSrc->points[idx].rgb = cloudSrc->points[idx].rgb;
            pcl::PointXYZRGB p;
            p.x = PointDest[0] + Centroid[0];
            p.y = PointDest[1] + Centroid[1];
            p.z = PointDest[2] + Centroid[2];
            p.rgb = cloudSrc->points[idx].rgb;
            //p.x *= 5; p.y *= 5; p.z *= 5;
            cloudTrans->push_back(p);
            //cloudTrans->points[idx].rgb = cloudSrc->points[idx].rgb;
        }

        return 1.0;
    }

    /*
    //获取点云，直接从上一步获取
    cv::Mat  getViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr  surface,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudView)
    {
        //获取大包围盒
        int bx, by, bz;
        std::vector<std::pair<float, float> > minmax(3);
        MathCal::findMinMax(surface, minmax);
        float xmin = minmax[0].first;
        float ymin = minmax[1].first;
        float zmin = minmax[2].first;
        bx = ceil(minmax[0].second - minmax[0].first);
        by = ceil(minmax[1].second - minmax[1].first);
        bz = ceil(minmax[2].second - minmax[2].first);
        std::vector<float> bbx;
        bbx.push_back(bx); bbx.push_back(by); bbx.push_back(bz);

        //std::vector<bool > visibies(surface->points.size() );//直接重新生成点，不取浮点数
        //生成图像//使用OpenCV画出对应灰度图片
        cv::Mat img = cv::Mat::zeros(by + 1, bx + 1, CV_32FC1);
        //for ( pcl::PointXYZ p: surface->points )
        for (int i = 0; i < surface->points.size(); ++i)
        {
            pcl::PointXYZ p = surface->points[i];
            int x = p.x - xmin;
            int y = p.y - ymin;

            float z = p.z - zmin + 1;
            //取最大Z//必须使用四邻域

            int x1 = floor(x); int x2 = ceil(x); //if (x1 < 0) x1 = 0;
            int y1 = floor(y); int y2 = ceil(y); //if (y1 < 0) y1 = 0;
            MathCal::cutValue(x1, 0, img.cols - 1);
            MathCal::cutValue(x2, 0, img.cols - 1);
            MathCal::cutValue(y1, 0, img.rows - 1);
            MathCal::cutValue(y2, 0, img.rows - 1);
            if ( img.at<float>(y1, x1) < z) img.at<float>(y1, x1) = z;
            if ( img.at<float>(y1, x2) < z) img.at<float>(y1, x2) = z;
            if ( img.at<float>(y2, x2) < z) img.at<float>(y2, x2) = z;
            if ( img.at<float>(y2, x1) < z) img.at<float>(y2, x1) = z;
        }


        //补加原始点云的四邻域//原始点云已添加，不再重复补偿，原始点云已删除
        pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudFourNear(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < cloud->points.size(); ++i)
        {
            pcl::PointXYZ p = cloud->points[i];
            float x = p.x - xmin;
            float y = p.y - ymin;
            float z = p.z - zmin + 1;
            int x1 = floor(x); int x2 = ceil(x); //if (x1<0) x1 = 0; if (x2<0) x2 = 0;
            int y1 = floor(y); int y2 = ceil(y); //if (y1<0) y1 = 0; if (y2<0) y2 = 0;
            MathCal::cutValue(x1, 0, img.cols - 1);
            MathCal::cutValue(x2, 0, img.cols - 1);
            MathCal::cutValue(y1, 0, img.rows - 1);
            MathCal::cutValue(y2, 0, img.rows - 1);
            //重复填充四邻域
            //若未被填充，则填充
            if ( 0.0001> img.at<float>(y1, x1) ) img.at<float>(y1, x1) = z;
            if (0.0001> img.at<float>(y1, x2)) img.at<float>(y1, x2) = z;
            if (0.0001> img.at<float>(y2, x2)) img.at<float>(y2, x2) = z;
            if (0.0001> img.at<float>(y2, x1)) img.at<float>(y2, x1) = z;

        }

        cloudView->resize(0);
        cv::Mat imgGray = cv::Mat::zeros(by + 1, bx + 1, CV_8UC1);
        float x, y, z;
        for (int i = 0; i < img.rows; ++i)
        {
            float *ptr = img.ptr<float>(i);
            unsigned char *ptrg = imgGray.ptr<unsigned char>(i);
            for (int j = 0; j < img.cols; ++j)
            {
                if (*ptr > 0)
                {
                    x = j - xmin;
                    y = i - ymin;
                    z = *ptr - zmin-1;
                    cloudView->points.push_back(pcl::PointXYZ(x, y, z));
                    if (z < 0) z = 0;
                    if (z >255) z = 255;
                    *ptrg = (unsigned char)z;
                }
                ++ptr;
                ++ptrg;
            }
        }

        cloudView->height = 1;
        cloudView->width = cloudView->points.size();

        //cv::flip(imgGray, imgGray, 2);
        //cv::imshow("imgGray", imgGray);
        //cv::waitKey(0);

        return imgGray;
    }
*/

    //获取点云，直接从上一步获取
    cv::Mat  getViewer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr  surface//,
        //const pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud,
        //pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudView
                       )
    {
        //获取大包围盒
        int bx, by, bz;
        std::vector<std::pair<float, float> > minmax(0);
        float scale = 100;
        std::vector<float> bbx = MathCal::findMinMax(surface, minmax,scale);
        float xmin = minmax[0].first;
        float ymin = minmax[1].first;
        float zmin = minmax[2].first;
        bx = bbx[0];
        by = bbx[1];
        bz = bbx[2];

        int edge =3;
        //std::vector<bool > visibies(surface->points.size() );//直接重新生成点，不取浮点数
        //生成图像//使用OpenCV画出对应灰度图片，用来标记取哪个z值；
        cv::Mat img = cv::Mat::zeros(by + edge, bx + edge, CV_32FC1);
        //同时生成彩色图像，根据z值进行填充
        //cv::Mat imgRGB = cv::Mat::zeros(by + 1, bx + 1, CV_8UC3);
        int c = 32;
        cv::Scalar color(c,c,c);
        cv::Mat imgRGB = cv::Mat(by + edge, bx + edge, CV_8UC3,color);

        bool dilated = true;
        //for ( pcl::PointXYZ p: surface->points )
        for (int i = 0; i < surface->points.size(); ++i)
        {
            pcl::PointXYZRGB p = surface->points[i];
            int x = (p.x - xmin)*scale;
            int y = (p.y - ymin)*scale;
            //float z = p.z - zmin*scale + 1;
            float z = p.z- zmin;
            //取最大Z//必须使用四邻域

            int x1 = floor(x); int x2 = ceil(x); //if (x1 < 0) x1 = 0;
            int y1 = floor(y); int y2 = ceil(y); //if (y1 < 0) y1 = 0;
            x1 +=1;x2 +=1; y1 +=1;y2 +=1;
            MathCal::cutValue(x1, 0, img.cols - 1);
            MathCal::cutValue(x2, 0, img.cols - 1);
            MathCal::cutValue(y1, 0, img.rows - 1);
            MathCal::cutValue(y2, 0, img.rows - 1);
//            if ( img.at<float>(y1, x1) < z) {
//                img.at<float>(y1, x1) = z;
//                imgRGB.at<cv::Vec3b>(y1, x1)[0] = p.r;
//                imgRGB.at<cv::Vec3b>(y1, x1)[1] = p.g;
//                imgRGB.at<cv::Vec3b>(y1, x1)[2] = p.b;
//            }
//            if ( img.at<float>(y1, x2) < z){
//                img.at<float>(y1, x2) = z;
//                imgRGB.at<cv::Vec3b>(y1, x2)[0] = p.r;
//                imgRGB.at<cv::Vec3b>(y1, x2)[1] = p.g;
//                imgRGB.at<cv::Vec3b>(y1, x2)[2] = p.b;
//            }
//            if ( img.at<float>(y2, x2) < z){
//                img.at<float>(y2, x2) = z;
//                imgRGB.at<cv::Vec3b>(y2, x2)[0] = p.r;
//                imgRGB.at<cv::Vec3b>(y2, x2)[1] = p.g;
//                imgRGB.at<cv::Vec3b>(y2, x2)[2] = p.b;
//            }
//            if ( img.at<float>(y2, x1) < z){
//                img.at<float>(y2, x1) = z;
//                imgRGB.at<cv::Vec3b>(y2, x1)[0] = p.r;
//                imgRGB.at<cv::Vec3b>(y2, x1)[1] = p.g;
//                imgRGB.at<cv::Vec3b>(y2, x1)[2] = p.b;
//            }
            if ( img.at<float>(y1, x1) < z) {
                img.at<float>(y1, x1) = z;
                imgRGB.at<cv::Vec3b>(y1, x1)[0] = p.b;
                imgRGB.at<cv::Vec3b>(y1, x1)[1] = p.g;
                imgRGB.at<cv::Vec3b>(y1, x1)[2] = p.r;
            }
            if ( img.at<float>(y1, x2) < z){
                img.at<float>(y1, x2) = z;
                imgRGB.at<cv::Vec3b>(y1, x2)[0] = p.b;
                imgRGB.at<cv::Vec3b>(y1, x2)[1] = p.g;
                imgRGB.at<cv::Vec3b>(y1, x2)[2] = p.r;
            }
            if ( img.at<float>(y2, x2) < z){
                img.at<float>(y2, x2) = z;
                imgRGB.at<cv::Vec3b>(y2, x2)[0] = p.b;
                imgRGB.at<cv::Vec3b>(y2, x2)[1] = p.g;
                imgRGB.at<cv::Vec3b>(y2, x2)[2] = p.r;
            }
            if ( img.at<float>(y2, x1) < z){
                img.at<float>(y2, x1) = z;
                imgRGB.at<cv::Vec3b>(y2, x1)[0] = p.b;
                imgRGB.at<cv::Vec3b>(y2, x1)[1] = p.g;
                imgRGB.at<cv::Vec3b>(y2, x1)[2] = p.r;
            }

            if(dilated){
                imgRGB.at<cv::Vec3b>(y2+1, x2)[0] = p.b;
                imgRGB.at<cv::Vec3b>(y2+1, x2)[1] = p.g;
                imgRGB.at<cv::Vec3b>(y2+1, x2)[2] = p.r;
                imgRGB.at<cv::Vec3b>(y2+1, x2+1)[0] = p.b;
                imgRGB.at<cv::Vec3b>(y2+1, x2+1)[1] = p.g;
                imgRGB.at<cv::Vec3b>(y2+1, x2+1)[2] = p.r;
                imgRGB.at<cv::Vec3b>(y2, x2+1)[0] = p.b;
                imgRGB.at<cv::Vec3b>(y2, x2+1)[1] = p.g;
                imgRGB.at<cv::Vec3b>(y2, x2+1)[2] = p.r;
                //                imgRGB.at<cv::Vec3b>(y2+2, x2)[0] = p.b;
                //                imgRGB.at<cv::Vec3b>(y2+2, x2)[1] = p.g;
                //                imgRGB.at<cv::Vec3b>(y2+2, x2)[2] = p.r;
                //                imgRGB.at<cv::Vec3b>(y2+2, x2+2)[0] = p.b;
                //                imgRGB.at<cv::Vec3b>(y2+2, x2+2)[1] = p.g;
                //                imgRGB.at<cv::Vec3b>(y2+2, x2+2)[2] = p.r;
                //                imgRGB.at<cv::Vec3b>(y2, x2+2)[0] = p.b;
                //                imgRGB.at<cv::Vec3b>(y2, x2+2)[1] = p.g;
                //                imgRGB.at<cv::Vec3b>(y2, x2+2)[2] = p.r;
                imgRGB.at<cv::Vec3b>(y1-1, x1)[0] = p.b;
                imgRGB.at<cv::Vec3b>(y1-1, x1)[1] = p.g;
                imgRGB.at<cv::Vec3b>(y1-1, x1)[2] = p.r;
                imgRGB.at<cv::Vec3b>(y1, x1-1)[0] = p.b;
                imgRGB.at<cv::Vec3b>(y1, x1-1)[1] = p.g;
                imgRGB.at<cv::Vec3b>(y1, x1-1)[2] = p.r;
                imgRGB.at<cv::Vec3b>(y1-1, x1-1)[0] = p.b;
                imgRGB.at<cv::Vec3b>(y1-1, x1-1)[1] = p.g;
                imgRGB.at<cv::Vec3b>(y1-1, x1-1)[2] = p.r;
            }
        }

        return imgRGB;
    }

};

class CloudPro
{
public:
    CloudPro(){};
    ~CloudPro(){};

public:


//    static bool init_view(){
//        if( is_debug)
//        {
//            static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("cloud Viewer1"));
//            static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_tmp (new pcl::visualization::PCLVisualizer ("cloud ViewerT"));
//            static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_src (new pcl::visualization::PCLVisualizer ("cloud Viewer_src"));
//            static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_dst (new pcl::visualization::PCLVisualizer ("cloud Viewer_dst"));
//        }
//        return true;
//    }

    static std::pair<float,float> find_z_wide( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud )
    {
        std::pair<float,float> zmaxmin;
        float zmin = 1000000.0;
        float zmax = -1000000.0;

        for(int i=0;i< cloud->points.size();++i){
            pcl::PointXYZRGB p = cloud->points[i];
            if(zmin >p.z ){
                zmin =p.z;
            }
            if( zmax <p.z ){
                zmax =p.z;
            }
        }
        g_floor_height  = zmin + g_thick;
        g_ceil_height = zmax - g_thick;

        zmaxmin.first = zmin;
        zmaxmin.second = zmax;

        return zmaxmin;
    }

    static int  cut_z( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cut )
    {

        for(int i=0;i< cloud->points.size();++i){
        //for(pcl::PointXYZRGB *p =cloud_cut->points.begin();p !=cloud_cut->points.end();++p){
            pcl::PointXYZRGB p = cloud->points[i];
            //std::cout<<" point z: "<< p.z<< " ..........." <<std::endl;
            //if(p.z< g_ceil_height  && p.z>g_floor_height ){
            if(p.z< g_ceil_height  ){
                pcl::PointXYZRGB p2(p) ;//= new pcl::PointXYZRGB();
                //pcl::PointXYZRGB p2 = new pcl::PointXYZRGB();
                cloud_cut->points.push_back(p2);
            }
        }

        return cloud_cut->points.size();
    }


    //获取特定高度的切片
    static int get_slice( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr slice ,float floor,float ceil)
    {
        for(int i=0;i< cloud->points.size();++i){
            pcl::PointXYZRGB p = cloud->points[i];
            if(p.z< ceil  && p.z>floor ){
                pcl::PointXYZRGB p2(p) ;//= new pcl::PointXYZRGB();
                slice->points.push_back(p2);
            }
        }

        return slice->points.size();
    }

    static int move_cloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ,float x, float y ,float z)
    {
        for(int i=0;i< cloud->points.size();++i)
        {
            cloud->points[i].x += x;
            cloud->points[i].y += y;
            cloud->points[i].z += z;
        }

        return 1;
    }

    static int move_cloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ,Eigen::Vector4f &Centroid)
    {
        for(int i=0;i< cloud->points.size();++i)
        {
            cloud->points[i].x += Centroid[0];
            cloud->points[i].y += Centroid[1];
            cloud->points[i].z += Centroid[2];
        }

        return 1;
    }

    static int move_cloud( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2,float x, float y ,float z)
    {
        for(int i=0;i< cloud->points.size();++i)
        {
            cloud2->points[i].x = cloud->points[i].x +x ;
            cloud2->points[i].y = cloud->points[i].y +y;
            cloud2->points[i].z = cloud->points[i].z +z;
            //cloud2->points[i].rgb = cloud->points[i].rgb ;
        }

        return 1;
    }

    static int triangle()
    {
        /*
      // Load input file into a PointCloud<T> with an appropriate type
      //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PCLPointCloud2 cloud_blob;
      //pcl::io::loadPCDFile ("..\\..\\source\\table_scene_lms400_downsampled.pcd", cloud_blob);
      pcl::fromPCLPointCloud2(cloud_blob, *cloud);
      //* the data should be available in cloud

      // Normal estimation*
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//设置法线估计对象
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);//存储估计的法线
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);//定义kd树指针
      tree->setInputCloud (cloud);//用cloud构造tree对象
      n.setInputCloud (cloud);//为法线估计对象设置输入点云
      n.setSearchMethod (tree);//设置搜索方法
      n.setKSearch (20);//设置k邻域搜素的搜索范围
      n.compute (*normals);//估计法线
      //* normals should not contain the point normals + surface curvatures

      // Concatenate the XYZ and normal fields*
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);//
      pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);//连接字段，cloud_with_normals存储有向点云
      //* cloud_with_normals = cloud + normals

      // Create search tree*
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
      tree2->setInputCloud (cloud_with_normals);//利用有向点云构造tree

      // Initialize objects
      pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
      pcl::PolygonMesh triangles;//存储最终三角化的网络模型

      // Set the maximum distance between connected points (maximum edge length)
      gp3.setSearchRadius (0.025);         //设置搜索半径radius，来确定三角化时k一邻近的球半径。

      // Set typical values for the parameters
      gp3.setMu (2.5);                     //设置样本点到最近邻域距离的乘积系数 mu 来获得每个样本点的最大搜索距离，这样使得算法自适应点云密度的变化
      gp3.setMaximumNearestNeighbors (100);//设置样本点最多可以搜索的邻域数目100 。
      gp3.setMaximumSurfaceAngle(M_PI/4);  //45 degrees，设置连接时的最大角度 eps_angle ，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
      gp3.setMinimumAngle(M_PI/18);        //10 degrees，设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
      gp3.setMaximumAngle(2*M_PI/3);       //120 degrees，设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
      gp3.setNormalConsistency(false);     //设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。

      // Get result
      gp3.setInputCloud (cloud_with_normals);//设置输入点云为有向点云
      gp3.setSearchMethod (tree2);           //设置搜索方式tree2
      gp3.reconstruct (triangles);           //重建提取三角化
     // std::cout << triangles;
      // Additional vertex information
      std::vector<int> parts = gp3.getPartIDs();//获得重建后每点的 ID, Parts 从 0 开始编号， a-1 表示未连接的点。

    //  获得重建后每点的状态，取值为 FREE 、 FRINGE 、 BOUNDARY 、 COMPLETED 、 NONE 常量，
    //  其中 NONE 表示未定义，
    //  FREE 表示该点没有在 三 角化后的拓扑内，为自由点，
    //  COMPLETED 表示该点在三角化后的拓扑内，并且邻域都有拓扑点，
    //  BOUNDARY 表示该点在三角化后的拓扑边缘，
    //  FRINGE 表示该点在 三 角化后的拓扑内，其连接会产生重叠边。

      std::vector<int> states = gp3.getPointStates();

      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer tri!"));
      viewer->setBackgroundColor (0, 0, 0);
      viewer->addPolygonMesh(triangles,"my");

      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
       // 主循环
      while (!viewer->wasStopped ())
      {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
        return cloud_pro->points.size();
        */

        return 0;
    }



    //依然不能处理彩色点云！！//是因为未添加库问题
    //template<typename T>
    //int filter_by_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered )
    //int filter_by_radius(pcl::PointCloud<T>::Ptr cloud,pcl::PointCloud<T>::Ptr cloud_filtered )
    static int filter_by_radius(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered )
    {

        //初始化点云
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered  (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCDWriter writer;

    //    //设置点云为500个，也可以自行修改，或者直接读取PCD文件
    //    cloud->width = 500;
    //    cloud->height = 1;
    //    cloud->points.resize(cloud->width * cloud->height);
    //    for (size_t i = 0; i < cloud->points.size(); ++i)
    //    {
    //      cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    //      cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    //      cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    //    }
        //writer.write<pcl::PointXYZ>("./original.pcd", *cloud, false);

        //if (strcmp(argv[1], "-r") == 0){
        if(true){
        //pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        //pcl::RadiusOutlierRemoval<T> outrem;
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
        // build the filter
        outrem.setInputCloud(cloud);
        //设置搜索半径为0.3
        outrem.setRadiusSearch(g_RadiusSearch);
        //在该半径中必须最少有5个邻点，则保持在点云中
        outrem.setMinNeighborsInRadius (5);
        // apply filter
        outrem.filter (*cloud_filtered);
        //writer.write<pcl::PointXYZ>("./points_outrem.pcd", *cloud_filtered, false);

        }else //if (strcmp(argv[1], "-c") == 0)
        {
    //      pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cloud(new pcl::ConditionAnd<pcl::PointXYZ> ());
    ////设置作用域为z，取大于0且小于0.8的位置，保留在点云中，其余进行移除
    //      range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
    //      range_cloud->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.8)));
    //      pcl::ConditionalRemoval<pcl::PointXYZ> condream;
    //      condream.setCondition(range_cloud);
    //      condream.setInputCloud(cloud);
    //      condream.setKeepOrganized(true);
    //      condream.filter(*cloud_filtered);
    //     // writer.write<pcl::PointXYZ>("./points_condream.pcd", *cloud_filtered, false);
        }
    //    else{
    //      std::cerr << "please specify command line arg '-r' or '-c' "<< std::endl;
    //      //exit(0);
    //    }

        return cloud_filtered->points.size();

        return 0;
    }

    //直通滤波
    static int filter_by_pass(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered )
    {
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fy(new pcl::PointCloud<pcl::PointXYZRGB>);
        //直通滤波
        {
            pcl::PassThrough<pcl::PointXYZRGB> pass_z;//设置滤波器对象
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fz(new pcl::PointCloud<pcl::PointXYZRGB>);

            //参数设置
            pass_z.setInputCloud(cloud);
            pass_z.setFilterFieldName("z");
            //z轴区间设置
            pass_z.setFilterLimits(2.3,6);
            //设置为保留还是去除
            pass_z.setFilterLimitsNegative(true);
            pass_z.filter(*cloud_fz);

            //直通滤波
            pcl::PassThrough<pcl::PointXYZRGB> pass_y;//设置滤波器对象
            //参数设置
            pass_y.setInputCloud(cloud_fz);
            pass_y.setFilterFieldName("y");
            //z轴区间设置
            pass_y.setFilterLimits(-1.5, 0.9);
            //pass_y.setFilterLimitsNegative(false);
            pass_y.filter(*cloud_filtered);
        }

       return 1;
    }

    //统计滤波
    static int filter_by_cal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered )
    {
        //show_cloud_rgb( cloud_fy,viewer_tmp );
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cal(new pcl::PointCloud<pcl::PointXYZRGB>);
        //统计滤波
        {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outrem;//创建统计滤波对象
            //参数设置
            outrem.setInputCloud(cloud);
            outrem.setMeanK(10);//附近邻近点数 初始化200 改成500慢了三倍20s，50为1.8s,20为1.1秒，依然很慢！
            outrem.setStddevMulThresh(1);//判断是否离群点
            outrem.filter(*cloud_filtered);
        }

       return 1;
    }

    //统计滤波
    static int filter_by_cal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered,int n_points )
    {
        //show_cloud_rgb( cloud_fy,viewer_tmp );
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cal(new pcl::PointCloud<pcl::PointXYZRGB>);
        //统计滤波
        {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outrem;//创建统计滤波对象
            //参数设置
            outrem.setInputCloud(cloud);
            outrem.setMeanK(n_points);//附近邻近点数 初始化200 改成500慢了三倍20s，50为1.8s,20为1.1秒，依然很慢！
            outrem.setStddevMulThresh(1);//判断是否离群点
            outrem.filter(*cloud_filtered);
        }

       return 1;
    }


    //依然不能处理彩色点云！！//是因为未添加库问题
    static int filter_by_votex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered )
    {
        //体素滤波
        {
            pcl::VoxelGrid<pcl::PointXYZRGB> vox;
                vox.setInputCloud(cloud);
                vox.setLeafSize(0.01f,0.01f,0.01f);//体素网格大小//使用1厘米滤波
                vox.filter(*cloud_filtered);
        }

       return 1;
    }

    //依然不能处理彩色点云！！//是因为未添加库问题
    static int filter_by_votex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered ,float vsize)
    {
        //体素滤波
        {
            pcl::VoxelGrid<pcl::PointXYZRGB> vox;
                vox.setInputCloud(cloud);
                vox.setLeafSize(vsize,vsize,vsize);//体素网格大小//使用1厘米滤波
                vox.filter(*cloud_filtered);
        }

       return 1;
    }


    //依然不能处理彩色点云！！//是因为未添加库问题
    static int filter_by_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered )
    {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fy(new pcl::PointCloud<pcl::PointXYZRGB>);
        //直通滤波
        {
            pcl::PassThrough<pcl::PointXYZRGB> pass_z;//设置滤波器对象
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fz(new pcl::PointCloud<pcl::PointXYZRGB>);

            //参数设置
            pass_z.setInputCloud(cloud);
            pass_z.setFilterFieldName("z");
            //z轴区间设置
            pass_z.setFilterLimits(2.3,6);
            //设置为保留还是去除
            pass_z.setFilterLimitsNegative(true);
            pass_z.filter(*cloud_fz);

            //直通滤波
            pcl::PassThrough<pcl::PointXYZRGB> pass_y;//设置滤波器对象
            //参数设置
            pass_y.setInputCloud(cloud_fz);
            pass_y.setFilterFieldName("y");
            //z轴区间设置
            pass_y.setFilterLimits(-1.5, 0.9);
            //pass_y.setFilterLimitsNegative(false);
            pass_y.filter(*cloud_fy);
        }
        //show_cloud_rgb( cloud_fy,viewer_tmp );
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cal(new pcl::PointCloud<pcl::PointXYZRGB>);
        //统计滤波
        {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outrem;//创建统计滤波对象
            //参数设置
            outrem.setInputCloud(cloud);
            outrem.setMeanK(200);//附近邻近点数
            outrem.setStddevMulThresh(1);//判断是否离群点
            outrem.filter(*cloud_cal);
        }
        //show_cloud_rgb( cloud_cal,viewer_tmp );
        //体素滤波
        {
            pcl::VoxelGrid<pcl::PointXYZRGB> vox;
                vox.setInputCloud(cloud_cal);
                vox.setLeafSize(0.01f,0.01f,0.01f);//体素网格大小//使用1厘米滤波
                vox.filter(*cloud_filtered);
        }

       return 1;
    }

    /*
    //依然不能处理彩色点云！！//是因为未添加库问题//不能使用模板库
    template<class T>
    int filter_by_votex(pcl::PointCloud<T>::Ptr cloud,pcl::PointCloud<T>::Ptr cloud_filtered )
    {

        pcl::PointCloud<T>::Ptr cloud_fy(new pcl::PointCloud<T>);
        //直通滤波
        {
            pcl::PassThrough<T> pass_z;//设置滤波器对象
            pcl::PointCloud<T>::Ptr cloud_fz(new pcl::PointCloud<T>);

            //参数设置
            pass_z.setInputCloud(cloud);
            pass_z.setFilterFieldName("z");
            //z轴区间设置
            pass_z.setFilterLimits(2.3,6);
            //设置为保留还是去除
            pass_z.setFilterLimitsNegative(true);
            pass_z.filter(*cloud_fz);

            //直通滤波
            pcl::PassThrough<T> pass_y;//设置滤波器对象
            //参数设置
            pass_y.setInputCloud(cloud_fz);
            pass_y.setFilterFieldName("y");
            //z轴区间设置
            pass_y.setFilterLimits(-1.5, 0.9);
            //pass_y.setFilterLimitsNegative(false);
            pass_y.filter(*cloud_fy);
        }
        //show_cloud_rgb( cloud_fy,viewer_tmp );
        pcl::PointCloud<T>::Ptr cloud_cal(new pcl::PointCloud<T>);
        //统计滤波
        {
            pcl::StatisticalOutlierRemoval<T> outrem;//创建统计滤波对象
            //参数设置
            outrem.setInputCloud(cloud);
            outrem.setMeanK(200);//附近邻近点数
            outrem.setStddevMulThresh(1);//判断是否离群点
            outrem.filter(*cloud_cal);
        }
        show_cloud_rgb( cloud_cal,viewer_tmp );
        //体素滤波
        {
            pcl::VoxelGrid<T> vox;
                vox.setInputCloud(cloud_cal);
                vox.setLeafSize(0.01f,0.01f,0.01f);//体素网格大小//使用1厘米滤波
                vox.filter(*cloud_filtered);
        }

       return 1;
    }
    */


    //寻找地板，找到地板，进行年全部填充；
    static int found_ground( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pro)
    {
         //using namespace std;
         //using namespace pcl;
        pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>()); //模型点云
        //pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());//模型角点
        //pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());  //目标点云
        //pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());//目标角点
        pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>()); //法线
        //pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
        //pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>()); //描述子
        //pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());

         //计算法线 normalestimationomp估计局部表面特性在每个三个特点，分别表面的法向量和曲率，平行，使用OpenMP标准。//初始化调度程序并设置要使用的线程数（默认为0）。
         pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
          norm_est.setKSearch(10);       //设置K邻域搜索阀值为10个点
          norm_est.setInputCloud(cloud);  //设置输入点云
         norm_est.compute(*model_normals);   //计算点云法线

         //norm_est.setInputCloud(scene);
         //norm_est.compute(*scene_normals);

#ifdef SHOW_CLOUD
         if (is_debug){
             viewer->setBackgroundColor (0, 0, 0);
             pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
             viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
             viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, model_normals, 10, 0.2, "normals");
             viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
             viewer->addCoordinateSystem (1.0);
             viewer->initCameraParameters ();
             viewer->spin();
         }
#endif

        return 1;
    }

    //使用预处理，使用滤波，三角化等预处理；
    //不能处理有色点云！//是因为未添加库问题
    static int prepro(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pro)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudT(new pcl::PointCloud<pcl::PointXYZRGB>);
        //filter_by_radius(cloud,cloud_pro);
        //filter_by_filter(cloud,cloud_pro);
        //filter_by_cal(cloud,cloudT);
        filter_by_votex(cloud,cloud_pro);//不能在体素滤波之后使用法线，不能进行kd-tree化。
        //found_ground(cloudT,cloud_pro);

        return 0;
    }

    //获取特定角度的单侧视图
    static cv::Mat  get_view( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_view,Eigen::Vector4f  &ViewPoint)
    {
        //show_cloud_rgb( cloud ,viewer_tmp );
        CViewExtract* viewer = new CViewExtract();

        //转换视图
        //Eigen::Vector4f  &ViewPoint,float alpha, float beta, float theta;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloudTrans(new pcl::PointCloud<pcl::PointXYZRGB>);
        viewer->viewTrans(cloud, cloudTrans, ViewPoint);

        //show_cloud_rgb( cloudTrans,viewer_dst );

        //正射投影
        cv::Mat imgRGB = viewer->getViewer(cloudTrans);
        if (is_debug) std::cout<< "getViewer over!" <<std::endl;

        return imgRGB;
    }


    static cv::Mat  get_view( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,Eigen::Vector4f  &ViewPoint)
    {
        //if(is_debug) init_view();
        std::pair<float,float> zminmax = find_z_wide(cloud);//寻找最大最小值，确定地板和天花板阈值，设置厚度为0.2即20厘米
        if(is_debug) std::cout << zminmax.first << " zmax:" <<zminmax.second <<std::endl;

#ifdef SHOW_CLOUD
        if(is_debug)
            show_cloud_rgb( cloud ,viewer_src);
#endif

        float flooor = g_floor_height -g_thick;
        float ceil =g_floor_height;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr slice(new pcl::PointCloud<pcl::PointXYZRGB>);
        slice->points.resize(0);
        //slice->points.reserve(cloud->points.size());
        //get_slice( cloud, slice , flooor, ceil);
        //show_cloud_rgb( slice,viewer_tmp );

        struct timeval timev;
        gettimeofday(&timev,NULL);
        int ts = timev.tv_sec*1000 + timev.tv_usec/1000;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cut(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_cut->points.resize(0);
        //cloud_cut->points.reserve(cloud->points.size());//容易造曾黑幕！不使用又造成Segmentation fault (core dumped)
        int p_num = cut_z(cloud,cloud_cut);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pro(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_pro->points.resize(0);
        cloud_pro->points.reserve(cloud->points.size());//
        p_num = prepro(cloud_cut,cloud_pro);


#ifdef SHOW_CLOUD
        if( is_debug )
            show_cloud_rgb( cloud_pro,viewer_tmp );
#endif

        //gettimeofday(&timev,NULL);
        //int te = timev.tv_sec*1000 + timev.tv_usec/1000;
        //std::cout<< "Time for per frame: " << te-ts  <<std::endl;

//        Eigen::Vector4f  ViewPoint;
//        float AngleA, AngleB, AngleC;//声明视角//初始化 作为原始角度
//        AngleA = 49.0/M_PI;// AngleTrans[0];//
//        AngleB = 78.9/M_PI;// AngleTrans[1];//
//        AngleC = 34.8/M_PI;// AngleTrans[2];//
//        ViewPoint<< AngleA, AngleB, AngleC,1;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_view;
        cv::Mat imgRGB = get_view(  cloud_pro, cloud_view,ViewPoint);
        //show_cloud_rgb( cloud_pro,viewer_dst );

        gettimeofday(&timev,NULL);
        int te2 = timev.tv_sec*1000 + timev.tv_usec/1000;
        std::cout<< "Time for per frame: " << te2-ts <<std::endl;

        return imgRGB;
    }//get view

};//class CloudPro

int pcd_icp(int argc, char *argv[])
{
    std::cout<< "helllo,wishchin! "<<std::endl;

    std::string filename,filename2,filename3;//(argv[1]);
    filename2 = std::string("data/cloud_002.pcd");
    filename3 = std::string("data/cloud_003.pcd");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_icp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);  // Transformed point cloud
    //pcl::io::loadPCDFile(filename ,*cloud);//Failed to find match for field 'rgb'.  why?!!!
    pcl::io::loadPCDFile(filename2 ,*cloud_in);
    pcl::io::loadPCDFile(filename3 ,*cloud_icp);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudf_in (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudf_icp (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudf_move (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::console::TicToc timef;
    timef.tic();
    float v_size =0.2;//体素化，选择体素大一些，生成大的体素
    int np =20;
    CloudPro::filter_by_votex(cloud_in,cloudf_in, v_size);
    CloudPro::filter_by_votex(cloud_icp,cloudf_icp, v_size);
    //CloudPro::filter_by_cal(cloud_in,cloudf_in, np);//time is so long,600ms!
    //CloudPro::filter_by_cal(cloud_icp,cloudf_icp, np);

    //1. move the Centroid, Give a better initation;
    Eigen::Vector4f CentroidIn,CentroidIcp,CentroidMove;
    CentroidIn << 0, 0, 0, 0;
    CentroidIcp << 0, 0, 0, 0;
    pcl::compute3DCentroid(*cloudf_in, CentroidIn);
    pcl::compute3DCentroid(*cloudf_icp, CentroidIcp);
    float mx,my,mz;
    //mx =CentroidIn[0] -CentroidIcp[0];
    CentroidMove =  CentroidIcp -CentroidIn;
    CloudPro::move_cloud( cloudf_in,CentroidMove);
    //pcl::compute3DCentroid(*cloudf_in, CentroidIn);

    //2. find a better rotate axis!



    // 创建 KD-Tree
    //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_in (new pcl::search::KdTree<pcl::PointXYZ>);
    //tree_in->setInputCloud (&cloud_in);   //用cloud构建tree对象

    std::cout << "filter time: " << timef.toc() << " ms" << std::endl;

    //show_cloud_rgb(cloud_in,viewer);
    //show_cloud_rgb( cloud_icp,viewer_tmp );
    show_cloud_rgb(cloudf_in,viewer);
    show_cloud_rgb( cloudf_icp,viewer_tmp );

    //viewer->spin();viewer_tmp->spin();

    pcl::console::TicToc time;
    {
        // Defining a rotation matrix and translation vector
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
        // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
        double theta = M_PI / 8;  // The angle of rotation in radians
        transformation_matrix (0, 0) = cos (theta);
        transformation_matrix (0, 1) = -sin (theta);
        transformation_matrix (1, 0) = sin (theta);
        transformation_matrix (1, 1) = cos (theta);
        // A translation on Z axis (0.4 meters)
        transformation_matrix (2, 3) = 0.4;
        // Display in terminal the transformation matrix
        std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
        print4x4Matrix (transformation_matrix);
        // Executing the transformation
        pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
        *cloud_ptr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
        // The Iterative Closest Point algorithm
        time.tic();

        int iterations =16;
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setMaximumIterations (iterations);
        icp.setInputSource (cloudf_icp);
        icp.setInputTarget (cloudf_in);
        icp.align (*cloudf_icp);
        icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
        std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

        show_cloud_rgb( cloudf_icp,cloudf_in,viewer_dst );

    }

    viewer->spin();viewer_tmp->spin();viewer_dst->spin();

    if(0){

    }

    return 1;
}

int cloud_pro(int argc, char *argv[])
{
    std::cout<< "helllo,wishchin! "<<std::endl;

    std::string filename;//(argv[1]);
    filename = std::string("/home/wishchin/map/wish/cloud_10243.pcd");
    filename = std::string("/home/wishchin/map/wish01/cloud_10244a.pcd");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(filename ,*cloud);//Failed to find match for field 'rgb'.  why?!!!

    Eigen::Vector4f  ViewPoint;
    float AngleA, AngleB, AngleC;//声明视角//初始化 作为原始角度
    AngleA = 49.0/M_PI;// AngleTrans[0];//
    AngleB = 78.9/M_PI;// AngleTrans[1];//
    AngleC = 34.8/M_PI;// AngleTrans[2];//
    AngleA = 60.0/M_PI;// AngleTrans[0];//
    AngleB = 60.0/M_PI;// AngleTrans[1];//
    AngleC = 60.0/M_PI;// AngleTrans[2];//
    AngleA = 60.0/1;// AngleTrans[0];//
    AngleB = 60.0/1;// AngleTrans[1];//
    AngleC = 60.0/1;// AngleTrans[2];//
//    AngleA = 90.0/M_PI;// AngleTrans[0];//
//    AngleB = 90.0/M_PI;// AngleTrans[1];//
//    AngleC = 90.0/M_PI;// AngleTrans[2];//
    //ViewPoint<< AngleA, AngleB, AngleC,1;
    ViewPoint<< AngleA, 0, 0,1;
    cv::Mat imgRGB = CloudPro::get_view(  cloud, ViewPoint );

    if(imgRGB.rows*imgRGB.cols>1)
    {
        cv::Mat ims;
        filename = std::string("/home/sarsee37/map/wish/cloud_10242.png");
        //cv::imwrite(filename,imgRGB);

        cv::resize(imgRGB,ims,cv::Size(540,960) );
        cv::imshow("imgRGB_trans!",ims);
        cv::waitKey(0);
    }

    return 0;
}
