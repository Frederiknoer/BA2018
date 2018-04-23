
//
// Created by fred on 3/12/18.
//

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/median_filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include "Algorithms.h"

using namespace std;
using namespace pcl;

#ifndef VIS_TEST_PCLPLANE_H
#define VIS_TEST_PCLPLANE_H


class PclPlane {
public:
    PclPlane();
    PclPlane(PointCloud<PointXYZ>::Ptr in_cloud);
    void insertCloud(std::vector<Algorithms::pts> in_cloud);

    void findPlane();
    float getDistToPlane(float x, float y, float z);
    void visualizeCloud(PointCloud<PointXYZ>::Ptr);
    void visualizeColorCloud(PointCloud<PointXYZRGB>::Ptr);
    void visualizePlaneCloud();
    PointCloud<PointXYZRGB>::Ptr mergeCloudsColor(PointCloud<PointXYZ>::Ptr cloud1, char color1, PointCloud<PointXYZ>::Ptr cloud2, char color2);
    PointCloud<PointXYZ>::Ptr removeOutliers(PointCloud<PointXYZ>::Ptr outlier_cloud, char);
    void mergeSort(PointCloud<PointXYZ>::Ptr,int size);




    ~PclPlane();
    PointCloud<PointXYZ>::Ptr input_cloud = {0};
    PointCloud<PointXYZ>::Ptr plane_cloud = {0};
    float coeffA = 0.0f, coeffB = 0.0f, coeffC = 0.0f, coeffD = 0.0f;
    PointCloud<PointXYZ>::Ptr coeff_cloud = {0};
    PointCloud<PointXYZ>::Ptr sorted_x = {0}, sorted_y = {0};

private:
    PointXYZ* mergeSortX(PointXYZ* arr, int l, int r);
    PointXYZ* mergeX(PointXYZ* arr, int l, PointXYZ* r, int m);
    PointXYZ* mergeSortY(PointXYZ* arr, int l, int r);
    PointXYZ* mergeY(PointXYZ* arr, int l, PointXYZ* r, int m);

};


#endif //VIS_TEST_PCLPLANE_H
