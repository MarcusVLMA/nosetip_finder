//
// Created by marcusv on 10/03/19.
//

#ifndef NOSETIP_FINDER_CROPPER_H
#define NOSETIP_FINDER_CROPPER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Cropper
{
    typedef pcl::PointCloud<pcl::PointXYZ>            CloudXYZ;
    typedef pcl::PointCloud<pcl::Normal>              CloudNormal;
    typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

    public:
        void static cropByPointIndex
        (
            CloudXYZ::ConstPtr &inputCloud,
            int pointIndex, std::string radiusOrK, float radiusOrKvalue,
            CloudXYZ::Ptr &outputCloud
        );

        void static cropByPointIndex
        (
            CloudXYZ::ConstPtr &inputCloud,
            CloudPC::Ptr &inputPrincipalCurvatures,
            int pointIndex, std::string radiusOrK, float radiusOrKvalue,
            CloudXYZ::Ptr &outputCloud,
            CloudPC::Ptr &outputPrincipalCurvaturesCloud,
            std::vector<int>& pointIndexSearch, std::vector<float>& pointSquaredDistance
        );

        void static cropByPointValues
        (
            CloudXYZ::Ptr &inputCloud,
            float xValue, float yValue, float zValue,
            std::string radiusOrK, float radiusOrKvalue,
            CloudXYZ::Ptr &outputCloud
        );

        void static cropByPointValues
        (
            CloudXYZ::Ptr &inputCloud,
            CloudPC::Ptr &inputPrincipalCurvatures,
            float xValue, float yValue, float zValue,
            std::string radiusOrK, float radiusOrKvalue,
            CloudXYZ::Ptr &outputCloud,
            CloudPC::Ptr &outputPrincipalCurvaturesCloud,
            std::vector<int> &pointIndexSearch, std::vector<float> &pointSquaredDistance
        );

        void static removeIsolatedPoints
        (
            CloudXYZ::Ptr &inputCloud,
            float radiusSearchSize, int pointsThreshold,
            CloudXYZ::Ptr &outputCloud
        );

        void static removeIsolatedPoints
        (
            CloudXYZ::Ptr &inputCloud,
            CloudPC::Ptr &inputPrincipalCurvatures,
            float radiusSearchSize, int pointsThreshold,
            CloudXYZ::Ptr &outputCloud,
            CloudPC::Ptr &outputPrincipalCurvaturesCloud
        );
    };


#endif //NOSETIP_FINDER_CROPPER_H
