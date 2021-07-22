#include <iostream>
#include <string>

// #include <pcl/io/ply_io.h>    //PLY相关头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> //
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT; //定义点云的格式

bool next_iteration = false;

void print4x4Matrix(const Eigen::Matrix4d &matrix) //打印旋转矩阵和平移矩阵
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 getRotMatrix()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

	// 随机填充无序点云
	cloud_in->width = 4;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);
	// for (size_t i = 0; i < cloud_in->points.size(); ++i) {
	//     cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
	//     cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
	//     cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	// }

	cloud_in->points[0].x = 435.779754638672;
	cloud_in->points[0].y = 115.793556213379;
	cloud_in->points[0].z = -137.312057495117;
	cloud_in->points[1].x = 435.918365478516;
	cloud_in->points[1].y = 74.018363952637;
	cloud_in->points[1].z = -136.977874755859;
	cloud_in->points[2].x = 394.927856445312;
	cloud_in->points[2].y = 74.891387939453;
	cloud_in->points[2].z = -141.068267822266;
	cloud_in->points[3].x = 394.861999511719;
	cloud_in->points[3].y = 115.015869140625;
	cloud_in->points[3].z = -141.323944091797;

	std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
			  << std::endl;
	for (size_t i = 0; i < cloud_in->points.size(); ++i)
		std::cout << "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z << std::endl;
	*cloud_out = *cloud_in;
	std::cout << "size:" << cloud_out->points.size() << std::endl;

	// // 在点云上执行简单的刚性变换，将cloud_out中的x平移0.7f米，然后再次输出数据值。
	// for (size_t i = 0; i < cloud_in->points.size(); ++i)
	//     cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
	// // 打印这些点

	cloud_out->points[1].x = 225.330459594727;
	cloud_out->points[1].y = 164.301330566406;
	cloud_out->points[1].z = 105.436019897461;
	cloud_out->points[0].x = 225.725494384766;
	cloud_out->points[0].y = 205.428527832031;
	cloud_out->points[0].z = 105.630218505859;
	cloud_out->points[3].x = 266.394958496094;
	cloud_out->points[3].y = 204.8898620605473;
	cloud_out->points[3].z = 99.470924377441;
	cloud_out->points[2].x = 265.878967285156;
	cloud_out->points[2].y = 164.728469848633;
	cloud_out->points[2].z = 99.314620971680;

	std::cout << "Transformed " << cloud_in->points.size() << " data points:"
			  << std::endl;
	for (size_t i = 0; i < cloud_out->points.size(); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
	// pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	// viewer.showCloud (cloud_in);
	// while (!viewer.wasStopped ())
	// {
	// }

	// 创建IterativeClosestPoint的实例
	// setInputSource将cloud_in作为输入点云
	// setInputTarget将平移后的cloud_out作为目标点云
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);
	icp.setMaximumIterations(10000);

	// 创建一个 pcl::PointCloud<pcl::PointXYZ>实例 Final 对象,存储配准变换后的源点云,
	// 应用 ICP 算法后, IterativeClosestPoint 能够保存结果点云集,如果这两个点云匹配正确的话
	// （即仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云）,那么 icp. hasConverged()= 1 (true),
	// 然后会输出最终变换矩阵的匹配分数和变换矩阵等信息。
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix = icp.getFinalTransformation();
	// std::cout << matrix << std::endl;
	return matrix;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
						   void *nothing)
{ //使用空格键来增加迭代次数，并更新显示
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

//------------------------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
	auto rotMatrix = getRotMatrix();
	std::cout << rotMatrix << std::endl;
	// 申明点云将要使用的
	PointCloudT::Ptr cloud_in(new PointCloudT);	 // 原始点云
	PointCloudT::Ptr cloud_tr(new PointCloudT);	 // 转换后的点云
	PointCloudT::Ptr cloud_icp(new PointCloudT); // ICP 输出点云

	// 检查程序输入命令的合法性
	if (argc < 2) //如果只有一个命令说明没有指定目标点云，所以会提示用法
	{
		printf("Usage :\n");
		printf("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
		PCL_ERROR("Provide one ply file.\n");
		return (-1);
	}

	int iterations = 1; // 默认的ICP迭代次数
	if (argc > 3)
	{
		//如果命令的有两个以上。说明用户是将迭代次数作为传递参数
		iterations = atoi(argv[2]); //传递参数的格式转化为int型
		if (iterations < 1)			//同时不能设置迭代次数为1
		{
			PCL_ERROR("Number of initial iterations must be >= 1\n");
			return (-1);
		}
	}

	pcl::console::TicToc time; //申明时间记录
	time.tic();				   //time.tic开始  time.toc结束时间
	if (pcl::io::loadPCDFile(argv[1], *cloud_in) < 0)
	{
		PCL_ERROR("Error loading cloud %s.\n", argv[1]);
		return (-1);
	}
	std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n"
			  << std::endl;

	if (pcl::io::loadPCDFile(argv[2], *cloud_icp) < 0)
	{
		PCL_ERROR("Error loading cloud %s.\n", argv[2]);
		return (-1);
	}
	std::cout << "\nLoaded file " << argv[2] << " (" << cloud_icp->size() << " points) in " << time.toc() << " ms\n"
			  << std::endl;

	//   //定义旋转矩阵和平移向量Matrix4d是为4*4的矩阵
	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity(); //初始化

	//   // 旋转矩阵的定义可以参考 ( https://en.wikipedia.org/wiki/Rotation_matrix)
	//   double theta = M_PI / 8;  // 旋转的角度用弧度的表示方法
	//   transformation_matrix (0, 0) = cos (theta);
	//   transformation_matrix (0, 1) = -sin (theta);
	//   transformation_matrix (1, 0) = sin (theta);
	//   transformation_matrix (1, 1) = cos (theta);

	//   // Z轴的平移向量 (0.4 meters)
	//   transformation_matrix (2, 3) = 0.4;

	//   //打印转换矩阵
	//   std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
	//   print4x4Matrix (transformation_matrix);

	//   // 执行点云转换
	//   pcl::transformPointCloud (*cloud_icp, *cloud_icp, rotMatrix);
	*cloud_tr = *cloud_icp; // 备份cloud_icp赋值给cloud_tr为后期使用

	// 迭代最近点算法
	time.tic(); //时间

	PointCloudT::Ptr cloud_src(new PointCloudT);
	PointCloudT::Ptr cloud_tar(new PointCloudT);

	// 随机填充无序点云
	cloud_src->width = 4;
	cloud_src->height = 1;
	cloud_src->is_dense = false;
	cloud_src->points.resize(cloud_in->width * cloud_in->height);
	// for (size_t i = 0; i < cloud_in->points.size(); ++i) {
	//     cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
	//     cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
	//     cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	// }

	cloud_src->points[0].x = 435.779754638672;
	cloud_src->points[0].y = 115.793556213379;
	cloud_src->points[0].z = -137.312057495117;
	cloud_src->points[1].x = 435.918365478516;
	cloud_src->points[1].y = 74.018363952637;
	cloud_src->points[1].z = -136.977874755859;
	cloud_src->points[2].x = 394.927856445312;
	cloud_src->points[2].y = 74.891387939453;
	cloud_src->points[2].z = -141.068267822266;
	cloud_src->points[3].x = 394.861999511719;
	cloud_src->points[3].y = 115.015869140625;
	cloud_src->points[3].z = -141.323944091797;

	std::cout << "Saved " << cloud_src->points.size() << " data points to input:"
			  << std::endl;
	for (size_t i = 0; i < cloud_src->points.size(); ++i)
		std::cout << "    " << cloud_src->points[i].x << " " << cloud_src->points[i].y << " " << cloud_src->points[i].z << std::endl;
	*cloud_tar = *cloud_src;
	std::cout << "size:" << cloud_tar->points.size() << std::endl;

	// // 在点云上执行简单的刚性变换，将cloud_tar中的x平移0.7f米，然后再次输出数据值。
	// for (size_t i = 0; i < cloud_in->points.size(); ++i)
	//     cloud_tar->points[i].x = cloud_in->points[i].x + 0.7f;
	// // 打印这些点

	cloud_tar->points[0].x = 435.779754638672 + 100;
	cloud_tar->points[0].y = 115.793556213379;
	cloud_tar->points[0].z = -137.312057495117;
	cloud_tar->points[1].x = 435.918365478516 + 100;
	cloud_tar->points[1].y = 74.018363952637;
	cloud_tar->points[1].z = -136.977874755859;
	cloud_tar->points[2].x = 394.927856445312 + 100;
	cloud_tar->points[2].y = 74.891387939453;
	cloud_tar->points[2].z = -141.068267822266;
	cloud_tar->points[3].x = 394.861999511719 + 100;
	cloud_tar->points[3].y = 115.015869140625;
	cloud_tar->points[3].z = -141.323944091797;

	std::cout << "Transformed " << cloud_tar->points.size() << " data points:"
			  << std::endl;
	for (size_t i = 0; i < cloud_tar->points.size(); ++i)
		std::cout << "    " << cloud_tar->points[i].x << " " << cloud_tar->points[i].y << " " << cloud_tar->points[i].z << std::endl;

	// *cloud_icp = *cloud_src;
	// *cloud_in = *cloud_tar;

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	//   icp.setRANSACIterations(1000);
	icp.setMaximumIterations(iterations); //设置最大迭代次数iterations=true

	icp.setInputSource(cloud_icp); //设置输入的点云
	icp.setInputTarget(cloud_in);  //目标点云
	icp.setTransformationEpsilon(1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon(1);

	icp.align(*cloud_icp);		 //匹配后源点云
	icp.setMaximumIterations(1); // 设置为1以便下次调用
	std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

	if (icp.hasConverged()) //icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		print4x4Matrix(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		return (-1);
	}

	// 可视化ICP的过程与结果
	pcl::visualization::PCLVisualizer viewer("ICP demo");
	// 创建两个观察视点
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	// 定义显示的颜色信息
	float bckgr_gray_level = 0.0; // Black
	float txt_gray_lvl = 1.0 - bckgr_gray_level;

	// 原始的点云设置为白色的
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
																			  (int)255 * txt_gray_lvl);
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1); //设置原始的点云都是显示为白色
	viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

	// 转换后的点云显示为绿色
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
	viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

	// ICP配准后的点云为红色
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
	viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

	// 加入文本的描述在各自的视口界面
	//在指定视口viewport=v1添加字符串“white 。。。”其中"icp_info_1"是添加字符串的ID标志，（10，15）为坐标16为字符大小 后面分别是RGB值
	viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
	viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

	std::stringstream ss;
	ss << iterations; //输入的迭代的次数
	std::string iterations_cnt = "ICP iterations = " + ss.str();
	viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

	// 设置背景颜色
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

	// 设置相机的坐标和方向
	viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
	viewer.setSize(1280, 1024); // 可视化窗口的大小

	// 注册按键回调函数
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);

	// 显示
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();

		//按下空格键的函数
		if (next_iteration)
		{
			// 最近点迭代算法
			time.tic();
			icp.align(*cloud_icp);
			std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

			if (icp.hasConverged())
			{
				printf("\033[11A"); // Go up 11 lines in terminal output.
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation().cast<double>(); // WARNING /!\ This is not accurate!
				print4x4Matrix(transformation_matrix);								  // 打印矩阵变换

				ss.str("");
				ss << iterations;
				std::string iterations_cnt = "ICP iterations = " + ss.str();
				viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
				viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
				return (-1);
			}
		}
		next_iteration = false;
	}
	return (0);
}