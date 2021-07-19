//
// Created by jin on 7/19/21.
//

#include "../include/Utils.h"
#include <iostream>


void Utils::print(){
    std::cout << "I am here " << std::endl;
}

bool Utils::LoadCloudFile(const std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file PCD format file \n");
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read file PLY format file \n");
            if (LoadCloudRGB(filename, point_cloud_ptr))
            {
                pcl::copyPointCloud(*point_cloud_ptr, *cloud);
                PCL_WARN("SUCCESS LOAD XYZ format cloud file \n");
            }
            else
            {
                PCL_ERROR("Couldn't read file XYZ format file \n");
                return false;
            }
        }
    }

    return true;


}


bool Utils::LoadCloudRGB(const std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr)
{
    ifstream fs;
    fs.open(filename.c_str(), ios::binary);
    if (!fs.is_open() || fs.fail())
    {
        PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(), strerror(errno));
        fs.close();
        return (false);
    }

    std::string line;
    std::vector<std::string> st;

    while (!fs.eof())
    {
        getline(fs, line);
        // Ignore empty lines
        if (line.empty())
            continue;

        // Tokenize the line
        boost::trim(line);
        boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

        // if (st.size() != 7)
        if (st.size() != 6)
            continue;

        // beacuse the point flippling around xy-axes
        pcl::PointXYZRGB point;
        point.x = float(atof(st[0].c_str()));
        point.y = float(atof(st[1].c_str())); // maybe minus
        point.z = float(atof(st[2].c_str())); // maybe minus
        point.r = int(atoi(st[3].c_str()));
        point.g = int(atoi(st[3].c_str()));
        point.b = int(atoi(st[3].c_str()));

        point_cloud_ptr->points.push_back(point);
    }
    fs.close();

    point_cloud_ptr->width = point_cloud_ptr->size();
    point_cloud_ptr->height = 1;
    std::cout << "the load cloud size :" << point_cloud_ptr->size() << std::endl;
    return (true);
}
