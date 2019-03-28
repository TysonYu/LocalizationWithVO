//
// Create by Tiezheng YU on 2019-2-25
//

#include <data_operator.h>
#include <MuInfo.h>

int main(int argc, char **argv)
{
    //使用的图像timestamp = 1403715274962142976 是在 name这个类中的第34个元素
    pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud (new pcl::PointCloud<pcl::PointXYZI>);//初始点云
    cv::Mat image_in(Params::camera_height, Params::camera_width, CV_8UC1);//相机图像
    cv::Mat RendImage(Params::camera_height, Params::camera_width, CV_8UC1);//render得到的图像
    //---------读取输入数据--------------------------------------------------------------
    original_cloud = loadPCD("../data/data.pcd");//读取点云
    LoadCameraCalibration("../data/mav0/cam0/sensor.yaml");//读取相机标定数据
    vector<string> image_name = LoadName("../data/mav0/cam0/data.csv");
    //---------对每一个图像和第一个图像对比得到MI的数值（测试MI的准确性用)--------------------------
    image_in = imread("../data/mav0/cam0/data/" + image_name.at(atoi(argv[1])) + ".png", 0);//读取图像
    // imwrite("130_raw_camera_V1.jpg",image_in);
    LoadGoundTruth(image_name.at(atoi(argv[1])));
    //---------处理图像畸变------------------------------------------------------------------
    cv::Mat refined_image = undistortion(image_in);//image_out为处理畸变之后的图像
    RendImage = PointCloud2IntensityImage(original_cloud, 0);
    // imwrite("130_raw_pose_V1.jpg",RendImage);

    // GradientAlgorithm(refined_image, original_cloud);
    // PrintPose();
    // RendImage = PointCloud2IntensityImage(original_cloud, 0);
    // equalizeHist(refined_image,refined_image);
    // imwrite("130_camera_V1.jpg",refined_image);
    cv::namedWindow("rend_image", CV_WINDOW_AUTOSIZE);
	cv::imshow("rend_image",RendImage);   //窗口中显示图像
    // cv::namedWindow("camera_image", CV_WINDOW_AUTOSIZE);
	// cv::imshow("camera_image",refined_image);   //窗口中显示图像
    waitKey(0);
    // cvDestroyWindow("rend_image");
    // cvDestroyWindow("camera_image");
    ShowResult(original_cloud, refined_image);//在点云中投影相机图像
    // imwrite("130_refined_pose_V1.jpg",RendImage);
    




    // pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud (new pcl::PointCloud<pcl::PointXYZI>);//初始点云
    // cv::Mat refined_image(Params::camera_height, Params::camera_width, CV_8UC1);//相机图像
    // cv::Mat RendImage(Params::camera_height, Params::camera_width, CV_8UC1);//render得到的图像
    // LoadCameraCalibration("../data/mav0/cam0/sensor.yaml");//读取相机标定数据
    // original_cloud = loadPCD("../data/data.pcd");//读取点云
    // refined_image = imread("L.jpg",0);
    // // cv::Mat refined_image = undistortion(image_in);//refined_image为处理畸变之后的图像
    // GradientAlgorithm(refined_image, original_cloud);
    // RendImage = PointCloud2IntensityImage(original_cloud, 0);
    // cv::namedWindow("rend_image", CV_WINDOW_AUTOSIZE);
	// cv::imshow("rend_image",RendImage);   //窗口中显示图像
    // cv::namedWindow("camera_image", CV_WINDOW_AUTOSIZE);
	// cv::imshow("camera_image",refined_image);   //窗口中显示图像
    // waitKey(0);
    // cvDestroyWindow("rend_image");
    // cvDestroyWindow("camera_image");
    // ShowResult(original_cloud, refined_image);//在点云中投影相机图像
    // PrintPose();
    return 0;
}
