//
// Create by tiezheng YU on 2019/2/25
//

#include <data_operator.h>
#include <MuInfo.h>
//---------process pcl cloud--------------------------------------------------
pcl::PointCloud<pcl::PointXYZI>::Ptr loadPCD(const string &pcd_file)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_file, *original_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file data.pcd \n");
    }
    return original_cloud;
}

void ViewPCLXYZI(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_xyz);//把PointXYZI转化为PointXYZ
    pcl::visualization::PCLVisualizer viewer("PCLXYZI Cloud viewer");
    viewer.addPointCloud(cloud_xyz, "sample cloud");
    viewer.setBackgroundColor(0,0,0);
    viewer.addCoordinateSystem();
    viewer.addCoordinateSystem();
    while(!viewer.wasStopped())
        // while (!viewer->wasStopped ())
        viewer.spinOnce();
}

void ViewPCLXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer viewer("PCLXYZ viewer");
    viewer.addPointCloud(cloud, "sample cloud");
    viewer.setBackgroundColor(0,0,0);
    viewer.addCoordinateSystem();
    viewer.addCoordinateSystem();
    while(!viewer.wasStopped())
        // while (!viewer->wasStopped ())
        viewer.spinOnce();
}

void ViewPCLXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer viewer("PCLXYZRGB viewer");
    viewer.addPointCloud(cloud, "sample cloud");
    viewer.setBackgroundColor(0,0,0);
    viewer.addCoordinateSystem();
    viewer.addCoordinateSystem();
    while(!viewer.wasStopped())
        // while (!viewer->wasStopped ())
        viewer.spinOnce();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudIntensity2RGB(pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i = 0; i < intensity_cloud->points.size(); i++)
    {
        pcl::PointXYZRGB point;
        point.x = intensity_cloud->points[i].x;
        point.y = intensity_cloud->points[i].y;
        point.z = intensity_cloud->points[i].z;
        point.r = intensity_cloud->points[i].intensity*255;
        point.g = intensity_cloud->points[i].intensity*255;
        point.b = intensity_cloud->points[i].intensity*255;
        rgb_cloud->points.push_back(point);
    }
    return rgb_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr world2Cam0(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
    float rt1 = 1 - 2*Params::q_RS_y*Params::q_RS_y - 2*Params::q_RS_z*Params::q_RS_z;
    float rt2 = 2*Params::q_RS_x*Params::q_RS_y - 2*Params::q_RS_w*Params::q_RS_z;
    float rt3 = 2*Params::q_RS_x*Params::q_RS_z + 2*Params::q_RS_w*Params::q_RS_y;
    float rt4 = Params::p_RS_R_x;
    float rt5 = 2*Params::q_RS_x*Params::q_RS_y + 2*Params::q_RS_w*Params::q_RS_z;
    float rt6 = 1 - 2*Params::q_RS_x*Params::q_RS_x - 2*Params::q_RS_z*Params::q_RS_z;
    float rt7 = 2*Params::q_RS_y*Params::q_RS_z - 2*Params::q_RS_w*Params::q_RS_x;
    float rt8 = Params::p_RS_R_y;
    float rt9 = 2*Params::q_RS_x*Params::q_RS_z - 2*Params::q_RS_w*Params::q_RS_y;
    float rt10 = 2*Params::q_RS_y*Params::q_RS_z + 2*Params::q_RS_w*Params::q_RS_x;
    float rt11 = 1 - 2*Params::q_RS_x*Params::q_RS_x - 2*Params::q_RS_y*Params::q_RS_y;
    float rt12 = Params::p_RS_R_z;
    float rt13 = 0;
    float rt14 = 0;
    float rt15 = 0;
    float rt16 = 1;
    Eigen::Matrix4f rt_body2world;
    Eigen::Matrix4f rt_cam02body;
    Eigen::Matrix4f rt;
    rt_body2world << rt1 , rt2 , rt3 , rt4 , rt5 , rt6 , rt7 , rt8 , rt9 , rt10 , rt11 , rt12 , rt13 , rt14 , rt15 , rt16;
    // rt_body2world << 1,0,0,rt4, 0,1,0,rt8, 0,0,1,rt12, 0,0,0,1;
    rt_cam02body << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix4f rt_world2body = rt_body2world.inverse();
    Eigen::Matrix4f rt_body2cam0 = rt_cam02body.inverse();
    rt = rt_body2cam0*rt_world2body;
    pcl::transformPointCloud (*input, *result, rt);//image，Z上未归一化的像素坐标系
    // std::cout << "rt_world2body = " << std::endl << rt_world2body << std::endl;
    // std::cout << "rt_body2cam0 = " << std::endl << rt_body2cam0 << std::endl; 
    return result;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr FilterFrontPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 50);//delete all the point that z<0 && z>40
    //pass.setFilterLimitsNegative (true);
    pass.filter (*result);
    return result;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetImageCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Matrix4f intrisic;//相机内参
    intrisic << Params::camera_fu, 0, Params::camera_cu, 0,    0, Params::camera_fv, Params::camera_cv, 0,    0, 0, 1, 0,    0, 0, 0, 1;
    pcl::transformPointCloud (*cloud, *result, intrisic);//result，Z上未归一化的像素坐标系
    for(int i = 0; i < result->points.size(); i++)
    {
        result->points[i].x = result->points[i].x / result->points[i].z;
        result->points[i].y = result->points[i].y / result->points[i].z;
        if(result->points[i].z > Params::max_depth)
            Params::max_depth = result->points[i].z;
    }
    // cout << "point_cloud_max_depth = " << Params::max_depth << endl;
    return result;
}
void ShowResult(pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud, cv::Mat camera_image)//在点云中投影相机图像
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//rgb点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cam0_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//cam0坐标系下rgb点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cam0_cloud_front (new pcl::PointCloud<pcl::PointXYZRGB>);//cam0坐标系下rgb点云(过滤掉后面的点)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//图像形式的点云
    rgb_cloud = PointCloudIntensity2RGB(original_cloud);//得到带有颜色信息的点云
    Cam0_cloud = world2Cam0(rgb_cloud);//在cam0坐标系下的点云
    Cam0_cloud_front = FilterFrontPoints(Cam0_cloud);//过滤掉相机后面的点
    image_cloud = GetImageCloud(Cam0_cloud_front);
    for(int i=0;i<image_cloud->points.size();i++)//把相机图片颜色投影到点云图上
    {

        if(image_cloud->points[i].x>=0  && image_cloud->points[i].x<Params::camera_width && image_cloud->points[i].y>=0 && image_cloud->points[i].y<Params::camera_height)
        {
            Cam0_cloud_front->points[i].r = camera_image.at<uchar>(round(image_cloud->points[i].y),round(image_cloud->points[i].x));
            Cam0_cloud_front->points[i].g = camera_image.at<uchar>(round(image_cloud->points[i].y),round(image_cloud->points[i].x));
            Cam0_cloud_front->points[i].b = camera_image.at<uchar>(round(image_cloud->points[i].y),round(image_cloud->points[i].x));
        }
    }
    pcl::PCDWriter writer;
	writer.writeBinary("calibration_data.pcd", *Cam0_cloud_front); 
    ViewPCLXYZRGB(Cam0_cloud_front);
}
//---------Load Calibration-----------------------------------------------------------------------------------------------------------
void LoadCameraCalibration(const string &camera_calibration)
{
    bool FSflag = false;
    cv::FileStorage fCalibration;
    FSflag = fCalibration.open(camera_calibration, cv::FileStorage::READ);
    if (FSflag == false) cout << "Cannot open the file" << endl;

    Params::camera_width = fCalibration["resolution"][0];
    Params::camera_height = fCalibration["resolution"][1];

    Params::camera_fu = fCalibration["intrinsics"][0];
    Params::camera_fv = fCalibration["intrinsics"][1];
    Params::camera_cu = fCalibration["intrinsics"][2];
    Params::camera_cv = fCalibration["intrinsics"][3];

    Params::distortion_coefficients[0] = fCalibration["distortion_coefficients"][0];
    Params::distortion_coefficients[1] = fCalibration["distortion_coefficients"][1];
    Params::distortion_coefficients[2] = fCalibration["distortion_coefficients"][2];
    Params::distortion_coefficients[3] = fCalibration["distortion_coefficients"][3];

    Params::p_RS_R_x = fCalibration["p_RS_R_x"];
    Params::p_RS_R_y = fCalibration["p_RS_R_y"];
    Params::p_RS_R_z = fCalibration["p_RS_R_z"];
    Params::q_RS_w = fCalibration["q_RS_w"];
    Params::q_RS_x = fCalibration["q_RS_x"];
    Params::q_RS_y = fCalibration["q_RS_y"];
    Params::q_RS_z = fCalibration["q_RS_z"];

    // std::cout << "camera_intrinsics = " << Params::camera_fu << "  " << Params::camera_fv << "  " << Params::camera_cu << "  " << Params::camera_cv << std::endl;
    // std::cout << "camera_extrinsics = " << std::endl;
    // for(int i = 0; i < 4; i++)
    // {
    //     for(int j = 0; j < 4; j++)
    //         std::cout << Params::camera_extrinsics[i][j] << "  ";
    //     std::cout << std::endl;
    // }
    // std::cout << "distortion_coefficients = " << Params::distortion_coefficients[0] << "  " << Params::distortion_coefficients[1] << "  " << Params::distortion_coefficients[2] << "  " << Params::distortion_coefficients[3] << std::endl;
}
vector<string> LoadName(const string &in_file)//输入所有图像的名字
{
    ifstream fin(in_file.c_str()); //打开文件流操作
    vector<string> fields; //声明一个字符串向量
    string line; 
	while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
		istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
            break;
		}
	}
    // for (int i = 0; i < fields.size(); i++)
    //     fields.at(i) = "../data/mav0/cam0/data/" + fields.at(i) + ".png";
    return fields;
}

void LoadGoundTruth (const string &camera_time_stamp)//输入GroundTruth(根据特定图像的序号)
{
    ifstream fin ("../data/mav0/state_groundtruth_estimate0/data.csv");
    vector<string> fields;
    string line;
    int count = 0;
    int flag = 0;
    while(getline(fin,line))
    {
        istringstream sin(line);
        string field;
        while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
		    fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
            if (fields.at(0) == camera_time_stamp)
            {
                flag = 1;
                fields.pop_back();
                for(int i = 0; i < 7; i++)
                    {
                        getline(sin, field, ',');
                        fields.push_back(field);
                    }
                // std::cout << count << std::endl;
            }
            else
            {
                fields.pop_back();
                break;
            }
            
		}
        if (flag == 1) break;
        count ++;
    }
    // for (int i = 0; i < fields.size(); i++)
    //     cout << fields.at(i) << endl;
    std::cout << camera_time_stamp << std::endl;
    Params::p_RS_R_x = stod(fields.at(0));
    Params::p_RS_R_y = stod(fields.at(1));
    Params::p_RS_R_z = stod(fields.at(2));
    Params::q_RS_w = stod(fields.at(3));
    Params::q_RS_x = stod(fields.at(4));
    Params::q_RS_y = stod(fields.at(5));
    Params::q_RS_z = stod(fields.at(6));
}
//--------process image ---------------------------------------------------------
Mat undistortion(Mat &input)
{
    Mat output = input.clone();
    Mat camera_matrix = Mat(3, 3, CV_32FC1);
    Mat distortion_coefficients = Mat(4,1, CV_32FC1);
    camera_matrix.at<float>(0,0) = Params::camera_fu;
    camera_matrix.at<float>(0,1) = 0;
    camera_matrix.at<float>(0,2) = Params::camera_cu;
    camera_matrix.at<float>(1,0) = 0;
    camera_matrix.at<float>(1,1) = Params::camera_fv;
    camera_matrix.at<float>(1,2) = Params::camera_cv;
    camera_matrix.at<float>(2,0) = 0;
    camera_matrix.at<float>(2,1) = 0;
    camera_matrix.at<float>(2,2) = 1;
    distortion_coefficients.at<float>(0) = Params::distortion_coefficients[0];
    distortion_coefficients.at<float>(1) = Params::distortion_coefficients[1];
    distortion_coefficients.at<float>(2) = Params::distortion_coefficients[2];
    distortion_coefficients.at<float>(3) = Params::distortion_coefficients[3];
    undistort(input, output, camera_matrix, distortion_coefficients);
    // cv::namedWindow("in",CV_WINDOW_AUTOSIZE);
    // cv::imshow("in", input);
    // cv::namedWindow("out",CV_WINDOW_AUTOSIZE);
    // cv::imshow("out", output);
    // cv::waitKey(0);
    // cout << camera_matrix << endl;
    // cout << distortion_coefficients << endl;
    return output;
}

cv::Mat GetIntensityImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    cv::Mat result(Params::camera_height, Params::camera_width, CV_8UC1);
    MatIterator_<uchar>Mbegin,Mend;
	for (Mbegin=result.begin<uchar>(),Mend=result.end<uchar>();Mbegin!=Mend;++Mbegin)
		*Mbegin=255 ;
    
	for(int i=0;i<cloud->points.size();i++)//把强度值投影到图像M上
    {

        if(cloud->points[i].x>=0  && cloud->points[i].x<Params::camera_width && cloud->points[i].y>=0 && cloud->points[i].y<Params::camera_height)
        {
            if( cloud->points[i].r < result.at<uchar>(cloud->points[i].y,cloud->points[i].x))
                result.at<uchar>(cloud->points[i].y,cloud->points[i].x) =  cloud->points[i].r;
        }
    }
    // namedWindow("Test", CV_WINDOW_AUTOSIZE); 
	// imshow("Test",result);   //窗口中显示图像
    // waitKey(0);
    return result;
}

cv::Mat GetIntensityImageDense(pcl::PointCloud<pcl::PointXYZRGB>::Ptr image)
{
    Mat M(Params::camera_height, Params::camera_width, CV_8UC1);//把点投影到M上
    Mat P(Params::camera_height, Params::camera_width, CV_8UC1);//扩展投影点
    Mat Depth(Params::camera_height, Params::camera_width, CV_8UC1);//创建保存深度的图像

	//遍历所有像素，初始化像素值
    MatIterator_<uchar>Mbegin,Mend;
	for (Mbegin=M.begin<uchar>(),Mend=M.end<uchar>();Mbegin!=Mend;++Mbegin)
		*Mbegin=255;
    for (Mbegin=Depth.begin<uchar>(),Mend=Depth.end<uchar>();Mbegin!=Mend;++Mbegin)
		*Mbegin=255;
	for(int i=0;i<image->points.size();i++)//把深度值投影到图像M上
    {
        if(image->points[i].x>=0  && image->points[i].x<Params::camera_width && image->points[i].y>=0 && image->points[i].y<Params::camera_height)
        {
            if( image->points[i].z/Params::max_depth*255 < Depth.at<uchar>(image->points[i].y,image->points[i].x))
            {
                Depth.at<uchar>(image->points[i].y,image->points[i].x) = image->points[i].z/Params::max_depth*255;
                M.at<uchar>(image->points[i].y,image->points[i].x) = image->points[i].r;
            }
        }
    }
    for(int count = 0; count < 10; count ++)
    {
        if (count%2 == 0) 
        {
            for (int i=1;i<M.rows-1;i++)
	        {
		        for (int j=1;j<M.cols-1;j++)
		        {
                    if(M.at<uchar>(i,j) == 255)
                    {
                        int temp = 255;
                        int sum = 0;
                        int cnt = 0;
                        for(int n = i-1; n < i+2; n++)
                        {
                            for(int m = j-1; m < j+2; m++)
                            {
                                if(M.at<uchar>(n,m) < temp )
                                {
                                    sum = sum + M.at<uchar>(n,m);
                                    cnt ++;
                                    temp = M.at<uchar>(n,m);
                                }   
                            }
                        }
                        if (cnt > 0) 
                            temp = sum / cnt;
                        P.at<uchar>(i,j) = temp;
                    }
                    else
                        P.at<uchar>(i,j)  = M.at<uchar>(i,j);
		        }
            }
        }
        else
        {
            for (int i=1;i<M.rows-1;i++)
	        {
		        for (int j=1;j<M.cols-1;j++)
		        {
                    if(P.at<uchar>(i,j) == 255)
                    {
                        int sum = 0;
                        int cnt = 0;
                        int temp = 255;
                        for(int n = i-1; n < i+2; n++)
                        {
                            for(int m = j-1; m < j+2; m++)
                            {
                                if(P.at<uchar>(n,m) < temp)
                                {
                                    sum = sum + P.at<uchar>(n,m);
                                    cnt ++;
                                    temp = P.at<uchar>(n,m);
                                }
                            }
                        }
                        if (cnt > 0) 
                            temp = sum / cnt;
                        M.at<uchar>(i,j) = temp;
                    }
                    else
                        M.at<uchar>(i,j)  = P.at<uchar>(i,j);
		        }
            }
        }
    }
    return M;
}

cv::Mat GetDepthImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cam0_cloud_front)
{
    Mat M(Params::camera_height, Params::camera_width, CV_8UC1);//把点投影到M上
    MatIterator_<uchar>Mbegin,Mend;//遍历所有像素，初始化像素值
	for (Mbegin=M.begin<uchar>(),Mend=M.end<uchar>();Mbegin!=Mend;++Mbegin)
		*Mbegin=255;
    for(int i=0;i<image_cloud->points.size();i++)//把深度值投影到图像M上
    {
        if(image_cloud->points[i].x>=0  && image_cloud->points[i].x<Params::camera_width && image_cloud->points[i].y>=0 && image_cloud->points[i].y<Params::camera_height)
        {
            if( Cam0_cloud_front->points[i].z/Params::max_depth*255 < M.at<uchar>(image_cloud->points[i].y,image_cloud->points[i].x) &&  Cam0_cloud_front->points[i].z/Params::max_depth*255 > 15)
                M.at<uchar>(image_cloud->points[i].y,image_cloud->points[i].x) = Cam0_cloud_front->points[i].z/Params::max_depth*255;
        }
    }
    return M;
}

cv::Mat GetDepthImageDense(pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cam0_cloud_front)
{
    Mat M(Params::camera_height, Params::camera_width, CV_8UC1);//把点投影到M上
    MatIterator_<uchar>Mbegin,Mend;//遍历所有像素，初始化像素值
	for (Mbegin=M.begin<uchar>(),Mend=M.end<uchar>();Mbegin!=Mend;++Mbegin)
		*Mbegin=255;
    for(int i=0;i<image_cloud->points.size();i++)//把深度值投影到图像M上
    {
        if(image_cloud->points[i].x>=0  && image_cloud->points[i].x<Params::camera_width && image_cloud->points[i].y>=0 && image_cloud->points[i].y<Params::camera_height)
        {
            if( Cam0_cloud_front->points[i].z/Params::max_depth*255 < M.at<uchar>(image_cloud->points[i].y,image_cloud->points[i].x) &&  Cam0_cloud_front->points[i].z/Params::max_depth*255 > 16)
                M.at<uchar>(image_cloud->points[i].y,image_cloud->points[i].x) = Cam0_cloud_front->points[i].z/Params::max_depth*255;
        }
    }
    Mat P(Params::camera_height, Params::camera_width, CV_8UC1);//扩展投影点
    for(int count = 0; count < 10; count ++)
    {
        if (count%2 == 0) 
        {
            for (int i=1;i<M.rows-1;i++)
	        {
		        for (int j=1;j<M.cols-1;j++)
		        {
                    if(M.at<uchar>(i,j) == 255 || M.at<uchar>(i,j) == 0)
                    {
                        int temp = 255;
                        int sum = 0;
                        int cnt = 0;
                        for(int n = i-1; n < i+2; n++)
                        {
                            for(int m = j-1; m < j+2; m++)
                            {
                                if(M.at<uchar>(n,m) < temp && M.at<uchar>(n,m)>0)
                                {
                                    sum = sum + M.at<uchar>(n,m);
                                    cnt ++;
                                    temp = M.at<uchar>(n,m);
                                }   
                            }
                        }
                        if (cnt > 0) 
                            temp = sum / cnt;
                        P.at<uchar>(i,j) = temp;
                    }
                    else
                        P.at<uchar>(i,j)  = M.at<uchar>(i,j);
		        }
            }
        }
        else
        {
            for (int i=1;i<M.rows-1;i++)
	        {
		        for (int j=1;j<M.cols-1;j++)
		        {
                    if(P.at<uchar>(i,j) == 255 || P.at<uchar>(i,j) == 0)
                    {
                        int sum = 0;
                        int cnt = 0;
                        int temp = 255;
                        for(int n = i-1; n < i+2; n++)
                        {
                            for(int m = j-1; m < j+2; m++)
                            {
                                if(P.at<uchar>(n,m) < temp && P.at<uchar>(n,m)>0)
                                {
                                    sum = sum + P.at<uchar>(n,m);
                                    cnt ++;
                                    temp = P.at<uchar>(n,m);
                                }
                            }
                        }
                        if (cnt > 0) 
                            temp = sum / cnt;
                        M.at<uchar>(i,j) = temp;
                    }
                    else
                        M.at<uchar>(i,j)  = P.at<uchar>(i,j);
		        }
            }
        }
    }
    return M;
}



//************************************************************************
// 函数名称:    	GetMutualInfo
// 访问权限:    	public 
// 创建日期:		2016/12/09
// 创 建 人:		
// 函数说明:		计算输入图像A和输入图像B的互信息
// 函数参数: 	cv::Mat & img_a	输入图像A
// 函数参数: 	cv::Mat & img_b	输入图像B
// 返 回 值:   	double
// 说明：       这个函数菜的一笔。。。。。。。。。。。。。。。。。。。。跑一张图要8秒钟
//************************************************************************
double GetMutualInfo(cv::Mat& img_a, cv::Mat& img_b)//计算NMI
{
	if (!img_a.data || !img_b.data)
	{
		cout << "no input img" << endl;
		return 0.0;
	}
	if (img_a.rows!=img_a.rows || img_a.cols!=img_b.cols)
	{
		cout << "input img's row and cosl not eqaul" << endl;
		return 0.0;
	}
 
	int rows(img_a.rows);
	int cols(img_b.cols);
	int total_pixel(rows*cols);
	unsigned char* data_a = NULL;
	unsigned char* data_b = NULL;
	double value(0.0);						//计算得到的互信息的结果
	double H_a(0.0), H_b(0.0), H_ab(0.0);	//图像A,B或是AB的信息熵
	double* count_array_a = new double[256];
	memset(count_array_a, 0, sizeof(double)*256);
	double* count_array_b = new double[256];
	memset(count_array_b, 0, sizeof(double)* 256);
	double* count_array_ab = new double[256*256];
	memset(count_array_ab, 0, sizeof(double)* 256 * 256);
    clock_t start = clock();
	//计算H_a, H_b
	for (int i=0; i<rows; i++)
	{
		data_a = img_a.ptr<unsigned char>(i);
		data_b = img_b.ptr<unsigned char>(i);
		for (int j=0; j<cols; j++)
		{
			count_array_a[data_a[j]]++;
			count_array_b[data_b[j]]++;
		}
	}
	for (int i=0; i<255; i++)
	{
		if (0.0 != count_array_a[i])
		{
			double p(count_array_a[i] / (double)total_pixel);
			H_a = H_a + (-1.0*p*(std::log(p) / std::log(2)));
		}
		
		if (0.0 != count_array_b[i])
		{
			double p(count_array_b[i] / (double)total_pixel);
			H_b = H_b + (-1.0*p*(std::log(p) / std::log(2)));
		}
	}
    clock_t H_aH_b = clock();
	//计算H_ab
	for (int m = 0; m < 256; m++)	//8位的灰度级
	{
		for (int n = 0; n < 256; n++)
		{
			for (int i = 0; i < rows; i++)
			{
				data_a = img_a.ptr<unsigned char>(i);
				data_b = img_b.ptr<unsigned char>(i);
				for (int j = 0; j < cols; j++)
				{
					//if ((std::abs(m-data_a[j])<20) && (std::abs(n-data_b[j])<20))
					if ((m == data_a[j]) && (n == data_b[j]))	//由联合概率密度的实际物理意义，统计点数
					{
						count_array_ab[m*256 + n]++;
					}
				}
			}
		}
	}
	for (int m = 0; m < 256; m++)
	{
		for (int n = 0; n < 256; n++)
		{
			if (0.0 != count_array_ab[m*256 + n])
			{
				double p(count_array_ab[m*256 + n] / (double)total_pixel);
				H_ab = H_ab + (-1.0*p*(std::log(p) / std::log(2)));
			}
		}
	}
	clock_t Hab = clock();
	value = (H_a + H_b) / H_ab;	//得出归一化互信息
	delete[] count_array_a;
	delete[] count_array_b;
	delete[] count_array_ab;
	count_array_ab = NULL;
	count_array_a = NULL;
	count_array_b = NULL;
    clock_t end = clock();
    // cout << "H_a and H_b cost time = " << (double)(end-H_aH_b)/CLOCKS_PER_SEC <<  " s"<< endl;
    // cout << "H_ab cost time = " << (double)(H_aH_b-Hab)/CLOCKS_PER_SEC <<  " s"<< endl;
    // cout << "one round cost time = " << (double)(end-start)/CLOCKS_PER_SEC <<  " s"<< endl;
	return value;
}
//---------直接从点云和pose得到一个向上插值的Intensity的深度图-------------------------------------------------------
cv::Mat PointCloud2IntensityImage(pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud, int flag)//从点云得到强度图
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//rgb点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cam0_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//cam0坐标系下rgb点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cam0_cloud_front (new pcl::PointCloud<pcl::PointXYZRGB>);//cam0坐标系下rgb点云(过滤掉后面的点)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//图像形式的点云
    cv::Mat RendImage(Params::camera_height, Params::camera_width, CV_8UC1);
    rgb_cloud = PointCloudIntensity2RGB(original_cloud);//得到带有颜色信息的点云
    Cam0_cloud = world2Cam0(rgb_cloud);//在cam0坐标系下的点云
    Cam0_cloud_front = FilterFrontPoints(Cam0_cloud);//过滤掉相机后面的点
    image_cloud = GetImageCloud(Cam0_cloud_front);
    if (flag == 0)
        RendImage = GetIntensityImageDense(image_cloud);
    else 
        RendImage = GetIntensityImage(image_cloud);
    return RendImage;
}
//---------直接从点云和pose得到一个向上插值的Depth的深度图-------------------------------------------------------
cv::Mat PointCloud2DepthImage(pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud, int flag)//从点云得到强度图
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//rgb点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cam0_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//cam0坐标系下rgb点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cam0_cloud_front (new pcl::PointCloud<pcl::PointXYZRGB>);//cam0坐标系下rgb点云(过滤掉后面的点)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);//图像形式的点云
    cv::Mat RendImage(Params::camera_height, Params::camera_width, CV_8UC1);
    rgb_cloud = PointCloudIntensity2RGB(original_cloud);//得到带有颜色信息的点云
    Cam0_cloud = world2Cam0(rgb_cloud);//在cam0坐标系下的点云
    Cam0_cloud_front = FilterFrontPoints(Cam0_cloud);//过滤掉相机后面的点
    image_cloud = GetImageCloud(Cam0_cloud_front);
    // ViewPCLXYZRGB(Cam0_cloud_front);
    if (flag == 0)
        RendImage = GetDepthImageDense(image_cloud, Cam0_cloud_front);
    else 
        RendImage = GetDepthImage(image_cloud, Cam0_cloud_front);
    return RendImage;
}
//---------梯度下降法来求互信息最大时的7个外参----------------------------------------------------------------------
void UpdatePose (double pose[])//更新相机pose的七个外参
{
    Params::p_RS_R_x = pose[0];
    Params::p_RS_R_y = pose[1];
    Params::p_RS_R_z = pose[2];
    Params::q_RS_w = pose[3];
    Params::q_RS_x = pose[4];
    Params::q_RS_y = pose[5];
    Params::q_RS_z = pose[6];
}

double norm2(double x[], double y[])//求2个输入的norm
{
    double result = 0;
    result  = sqrt(pow(x[0]-y[0],2) + pow(x[1]-y[1],2) + pow(x[2]-y[2],2) + pow(x[3]-y[3],2) + pow(x[4]-y[4],2) + pow(x[5]-y[5],2) + pow(x[6]-y[6],2));
    return result;
}
double norm1(double x[])//求1个输入的norm
{
    double result = 0;
    result  = sqrt(pow(x[0],2) + pow(x[1],2) + pow(x[2],2) + pow(x[3],2) + pow(x[4],2) + pow(x[5],2) + pow(x[6],2));
    return result;
}
void PrintPose()
{
    cout << "pose = " << Params::p_RS_R_x << "  " << Params::p_RS_R_y << "  " << Params::p_RS_R_z << "  " << Params::q_RS_w << "  " << Params::q_RS_x << "  " << Params::q_RS_y << "  " << Params::q_RS_z << endl;    
}
double Getgama (double s[], double g[])//计算gama
{
    double up = pow(s[0],2) + pow(s[1],2) + pow(s[2],2) + pow(s[3],2) + pow(s[4],2) + pow(s[5],2) + pow(s[6],2);
    double down = s[0]*g[0] + s[1]*g[1] + s[2]*g[2] + s[3]*g[3] + s[4]*g[4] + s[5]*g[5] + s[6]*g[6];
    return up/down;
}

// void GradientAlgorithm(cv::Mat refined_image, pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud)//论文中的方法，对于多局部最大的情况不适用
// {
//     double step[7] = {0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005};
//     double G_pre[7] = {0, 0, 0, 0, 0, 0, 0};
//     double G_cur[7] = {0, 0, 0, 0, 0, 0, 0};
//     double g[7] = {0, 0, 0, 0, 0, 0, 0};
//     double NMI_pre[7] = {0, 0, 0, 0, 0, 0, 0};
//     double NMI_cur[7] = {0, 0, 0, 0, 0, 0, 0};
//     double theta_pre[7] = {Params::p_RS_R_x, Params::p_RS_R_y, Params::p_RS_R_z, Params::q_RS_w, Params::q_RS_x, Params::q_RS_y, Params::q_RS_z};
//     double theta_cur[7] = {Params::p_RS_R_x, Params::p_RS_R_y, Params::p_RS_R_z, Params::q_RS_w, Params::q_RS_x, Params::q_RS_y, Params::q_RS_z};
//     // UpdatePose (theta_cur);
//     cout << "初始";
//     PrintPose();
//     int count = 0;
//     cv::Mat RendImage(Params::camera_height, Params::camera_width, CV_8UC1);
//     RendImage = PointCloud2IntensityImage(original_cloud, 0);
//     MuInfo MI;
//     MI.gray_images.push_back(refined_image);
//     MI.depth_images.push_back(RendImage);
//     NMI_pre[0]  = MI.GetCost();
//     MI.gray_images.clear();
//     MI.depth_images.clear();
//     cout << "NMI_cur " << " = " << NMI_pre[0] << endl;
//     for (int i  = 0; i < 7; i ++)
//     {
//         NMI_pre[i] = NMI_pre[0];
//         theta_cur[i] = theta_pre[i] + step[i];
//         UpdatePose (theta_cur);
//         PrintPose();
//         RendImage = PointCloud2IntensityImage(original_cloud, 0);
//         theta_cur[i] = theta_cur[i] - step[i];
//         UpdatePose (theta_cur);
//         MI.gray_images.push_back(refined_image);
//         MI.depth_images.push_back(RendImage);
//         NMI_cur[i]  = MI.GetCost();
//         MI.gray_images.clear();
//         MI.depth_images.clear();
//         G_cur[i] = (NMI_cur[i] - NMI_pre[i]) / (step[i]);
//         g[i] = G_cur[i] - G_pre[i];
//         // cout << "g " << i << " = " << g[i] << endl;
//         cout << "NMI_cur " << i << " = " << NMI_cur[i] << endl;
//     }
//     for(int i = 0; i < 7; i++)
//     {
//         theta_cur[i] = theta_pre[i] + step[i];
//         // step[i] = (Getgama(step, g) * (G_cur[i] / norm1(G_cur)));
//         step[i] = (G_cur[i] / norm1(G_cur)) / 50;
//     }
//     // cout << " gama = " << Getgama(step, g) << endl;
//     cout << "step = " << step[0] << "  " << step[1] << "  " << step[2] << "  " << step[3] << "  " << step[4] << "  " << step[5] << "  " << step[6] << endl;    
//     UpdatePose (theta_cur);

//     while(norm1(step) > 0.005)
//     {
//         count ++;
//         for(int i = 0; i < 7; i++)
//         {
//             NMI_pre[i] = NMI_cur[i];
//             theta_pre[i] = theta_cur[i];
//             G_pre[i] = G_cur[i];
//         }
//         for (int i  = 0; i < 7; i ++)
//         {
//             theta_cur[i] = theta_pre[i] + step[i];
//             UpdatePose (theta_cur);
//             PrintPose();
//             RendImage = PointCloud2IntensityImage(original_cloud, 0);
//             theta_cur[i] = theta_cur[i] - step[i];
//             UpdatePose (theta_cur);
//             MI.gray_images.push_back(refined_image);
//             MI.depth_images.push_back(RendImage);
//             NMI_cur[i]  = MI.GetCost();
//             MI.gray_images.clear();
//             MI.depth_images.clear();
//             G_cur[i] = (NMI_cur[i] - NMI_pre[i]) / (step[i]);
//             g[i] = G_cur[i] - G_pre[i];
//             // cout << "g " << i << " = " << g[i] << endl;
//             cout << "NMI_cur[i] =  " << NMI_cur[i] << endl;
//         }

//         for(int i = 0; i < 7; i++)
//         {
//             theta_cur[i] = theta_pre[i] + step[i];
//             // step[i] = (Getgama(step, g) * (G_cur[i] / norm1(G_cur)));
//             step[i] = (G_cur[i] / norm1(G_cur)) / 70;
//         }
//         // cout << " gama = " << Getgama(step, g) << endl;
//         cout << "step = " << step[0] << "  " << step[1] << "  " << step[2] << "  " << step[3] << "  " << step[4] << "  " << step[5] << "  " << step[6] << endl;
//         UpdatePose (theta_cur);    
//         PrintPose();
//         cout << endl;
//         if(count > 50)
//             break;
//     }
// }

void GradientAlgorithm(cv::Mat refined_image, pcl::PointCloud<pcl::PointXYZI>::Ptr original_cloud)//通过输入一个标准图像，得到最佳情况下的7个外参
{
    double step[7] = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
    double NMI_pre = 0;
    double NMI_cur = 0;
    double G_pre = 0;
    double G_cur = 0;
    double theta_pre[7] = {Params::p_RS_R_x, Params::p_RS_R_y, Params::p_RS_R_z, Params::q_RS_w, Params::q_RS_x, Params::q_RS_y, Params::q_RS_z};
    double theta_cur[7] = {Params::p_RS_R_x, Params::p_RS_R_y, Params::p_RS_R_z, Params::q_RS_w, Params::q_RS_x, Params::q_RS_y, Params::q_RS_z};
    int flag1[7] = {0, 0, 0, 0, 0, 0, 0};
    int flag2[7] = {0, 0, 0, 0, 0, 0, 0};
    int flag3[7] = {0, 0, 0, 0, 0, 0, 0};
    int flag4[7] = {0, 0, 0, 0, 0, 0, 0};
    int flag5[7] = {0, 0, 0, 0, 0, 0, 0};
    int flag6[7] = {0, 0, 0, 0, 0, 0, 0};
    int count = 0;
    double step_size = 1;
    cv::Mat RendImage(Params::camera_height, Params::camera_width, CV_8UC1);

    RendImage = PointCloud2IntensityImage(original_cloud, 0);
    MuInfo MI;
    MI.gray_images.push_back(refined_image);
    MI.depth_images.push_back(RendImage);
    NMI_pre  = MI.GetCost();
    MI.gray_images.clear();
    MI.depth_images.clear();
    cout << "NMI_cur =  " << NMI_pre << endl;
    for (int i  = 0; i < 7; i ++)
    {
        theta_cur[i] = theta_pre[i] + step[i];
        UpdatePose (theta_cur);
        PrintPose();
        RendImage = PointCloud2IntensityImage(original_cloud, 0);
        MI.gray_images.push_back(refined_image);
        MI.depth_images.push_back(RendImage);
        NMI_cur  = MI.GetCost();
        MI.gray_images.clear();
        MI.depth_images.clear();
        G_cur = NMI_cur - NMI_pre;
        if (NMI_cur < NMI_pre)
            step[i] = - step[i];
        NMI_pre = NMI_cur;
        cout << "step " << i << " = " << step[i] << endl;
        cout << "NMI_cur =  " << NMI_cur << endl;
    }

    cout << "step = " << step[0] << "  " << step[1] << "  " << step[2] << "  " << step[3] << "  " << step[4] << "  " << step[5] << "  " << step[6] << endl;    

    while(1)
    {
        count ++;
        for(int i = 0; i < 7; i++)
        {
            G_pre = G_cur;
            if(count > 170 && flag1[i] == 0)
            {
                flag1[i] = 1;
                step[i] = step[i]/2;
            }
            if(count > 220 && flag2[i] == 0)
            {
                flag2[i] = 1;
                step[i] = step[i]/2;
            }
            // if(count > 80 && flag3[i] == 0)
            // {
            //     flag3[i] = 1;
            //     step[i] = step[i]/2;
            // }
            // if(count > 90 && flag4[i] == 0)
            // {
            //     flag4[i] = 1;
            //     step[i] = step[i]/2;
            // }
            // if(count > 90 && flag5[i] == 0)
            // {
            //     flag5[i] = 1;
            //     step[i] = step[i]/2;
            // }
            // if(count > 95 && flag6[i] == 0)
            // {
            //     flag6[i] = 1;
            //     step[i] = step[i]/2;
            // }
            theta_pre[i] = theta_cur[i];
        }
        NMI_pre = NMI_cur;
        for (int i  = 0; i < 7; i ++)
        {
            theta_cur[i] = theta_pre[i] + step[i];
            UpdatePose (theta_cur);
            // PrintPose();
            RendImage = PointCloud2IntensityImage(original_cloud, 0);
            MI.gray_images.push_back(refined_image);
            MI.depth_images.push_back(RendImage);
            NMI_cur  = MI.GetCost();
            MI.gray_images.clear();
            MI.depth_images.clear();
            G_cur = NMI_cur - NMI_pre;
            if (NMI_cur < NMI_pre)
            {
                if (abs(G_cur) > abs(G_pre))
                    step[i] = - step[i] * step_size;
                else
                    step[i] = - step[i] / step_size;
            }
            else
            {
                if (abs(G_cur) > abs(G_pre))
                    step[i] =  step[i] * step_size;
                else
                    step[i] =  step[i] / step_size;
            }
            NMI_pre = NMI_cur;
            // cout << "step " << i << " = " << step[i] << endl;
            cout << "NMI_cur =  " << NMI_cur << endl;
        }
        cout << "step = " << step[0] << "  " << step[1] << "  " << step[2] << "  " << step[3] << "  " << step[4] << "  " << step[5] << "  " << step[6] << endl;
        std::cout << "count = " << count << std::endl;
        if(count > 270)
            break;
    }
}


