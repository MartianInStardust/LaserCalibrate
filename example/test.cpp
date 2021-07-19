//
// Created by jin on 7/19/21.
//

#include <iostream>
#include <memory>
#include "../include/Utils.h"
#include <pcl/visualization/cloud_viewer.h>



int main(int argc, char **argv)
{
    std::shared_ptr<Utils> pcl_utility = std::make_shared<Utils>();


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (!pcl_utility->LoadCloudFile(argv[1], cloud))
    {
        std::cerr << "failed load cloud file!!";
        return (-1);
    }

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ())
    {
    }
    return 0;
}
