//
// Created by jin on 7/19/21.
//

#include <iostream>
#include <memory>
#include "../include/Utils.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int main(int argc, char **argv)
{
    //    std::shared_ptr<Utils> pcl_utility = std::make_shared<Utils>();
    //
    //
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //
    //    if (!pcl_utility->LoadCloudFile(argv[1], cloud))
    //    {
    //        std::cerr << "failed load cloud file!!";
    //        return (-1);
    //    }
    //
    //    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //    viewer.showCloud (cloud);
    //    while (!viewer.wasStopped ())
    //    {
    //    }
    //    return 0;

    // 定义输入和输出点云
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

    cloud_in->points[0].x =
        435.779754638672;
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

    cloud_out->points[0].x = 225.330459594727;
    cloud_out->points[0].y = 164.301330566406;
    cloud_out->points[0].z = 105.436019897461;
    cloud_out->points[1].x = 225.725494384766;
    cloud_out->points[1].y = 205.428527832031;
    cloud_out->points[1].z = 105.630218505859;
    cloud_out->points[2].x = 266.394958496094;
    cloud_out->points[2].y = 204.8898620605473;
    cloud_out->points[2].z = 99.470924377441;
    cloud_out->points[3].x = 265.878967285156;
    cloud_out->points[3].y = 164.728469848633;
    cloud_out->points[3].z = 99.314620971680;

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

    // 创建一个 pcl::PointCloud<pcl::PointXYZ>实例 Final 对象,存储配准变换后的源点云,
    // 应用 ICP 算法后, IterativeClosestPoint 能够保存结果点云集,如果这两个点云匹配正确的话
    // （即仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云）,那么 icp. hasConverged()= 1 (true),
    // 然后会输出最终变换矩阵的匹配分数和变换矩阵等信息。
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix = icp.getFinalTransformation();
    std::cout << matrix << std::endl;
    return (0);
}
