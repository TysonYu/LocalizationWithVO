//
// Create by Tiezheng YU on 2019/2/25
//
#ifndef DATA_OPERATOR_H
#define DATA_OPERATOR_H

#include <Eigen/Core>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>//allows us to use pcl::transformPointCloud function
#include <pcl/filters/passthrough.h>//allows us to use pcl::PassThrough
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <boost/program_options.hpp>
#include <boost/program_options.hpp>

#include <string>
#include <iostream>
#include <math.h>
#include <Params.h>
#include <fstream>
#include <sstream>
using namespace pcl;
using namespace std;
using namespace cv;
using namespace Eigen;

//---------process pcl cloud---------------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZI>::Ptr loadPCD(const string &pcd_file);
void ViewPCLXYZI(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
void ViewPCLXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void ViewPCLXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudIntensity2RGB(pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud);//把强度信息转化为颜色信息
pcl::PointCloud<pcl::PointXYZRGB>::Ptr world2Cam0(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);//从世界坐标系转化到body坐标系
pcl::PointCloud<pcl::PointXYZRGB>::Ptr FilterFrontPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);//过滤剩下相机前面的点
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetImageCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
cv::Mat GetIntensityImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);//从RGB点云获得图像（需要相机内参）
cv::Mat GetIntensityImageDense(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);//从RGB点云获得稠密图像（需要相机内参）
cv::Mat GetDepthImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cam0_cloud_front);//从RGB点云获得深度图
cv::Mat GetDepthImageDense(pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cam0_cloud_front);//从RGB点云获得稠密深度图
void ShowResult(pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud, cv::Mat camera_image);//在点云中投影相机图像
//---------Load----------------------------------------------------------------------------
void LoadCameraCalibration(const string &camera_calibration);//输入相机标定文件
void LoadGoundTruth (const string &camera_time_stamp);//输入GroundTruth(根据特定图像的序号)
vector<string> LoadName(const string &in_file);//输入所有图像的名字
//---------porcess image -------------------------------------------------------------------------------------
Mat undistortion(Mat &input);//图像去除畸变
double CoEntorpy(Mat& inputImg1, Mat& inputImg2);//计算两个图像的互信息
double GetMutualInfo(cv::Mat& img_a, cv::Mat& img_b);//计算两个图像的归一化互信息
//---------直接从点云和pose得到一个向上插值的intensity的深度图-------------------------------------------------------
cv::Mat PointCloud2IntensityImage(pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud, int flag);//从点云得到强度图
cv::Mat PointCloud2DepthImage(pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud, int flag);//从点云得到强度图
//---------梯度下降法来求互信息最大时的7个外参----------------------------------------------------------------------
void UpdatePose (double pose[]);//更新相机pose的七个外参
double norm2(double x[], double y[]);//求2个输入的norm
double norm1(double x[]);//求1个输入的norm
void PrintPose();//输出当前姿态
double Getgama (double s[], double g[]);//计算gama_k
void GradientAlgorithm(cv::Mat refined_image, pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud);//通过输入一个标准图像，得到最佳情况下的7个外参

#endif //DATA_OPERATOR_H