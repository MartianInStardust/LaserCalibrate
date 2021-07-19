//
// Created by jin on 7/19/21.
//

#ifndef LASERCALIBRATE_UTILS_H
#define LASERCALIBRATE_UTILS_H

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>


class Utils {
public:
    void print();
    bool LoadCloudFile(const std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

};


#endif //LASERCALIBRATE_UTILS_H
