//
// Created by marcusv on 10/03/19.
//

#include "Cropper.h"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

void Cropper::cropByPointIndex(CloudXYZ::ConstPtr &inputCloud, int pointIndex, std::string radiusOrK,
                               float radiusOrKvalue, CloudXYZ::Ptr &outputCloud)
{
    std::vector<int> defaultPointIndexSearch;
    std::vector<float> defaultPointSquaredDistance;

    CloudPC::Ptr
        defaultPrincipalCurvaturesCloud(new CloudPC());

    defaultPrincipalCurvaturesCloud->points.resize(inputCloud->points.size());

    Cropper::cropByPointIndex(
        inputCloud, defaultPrincipalCurvaturesCloud, pointIndex, radiusOrK, radiusOrKvalue, outputCloud,
        defaultPrincipalCurvaturesCloud, defaultPointIndexSearch, defaultPointSquaredDistance);
}

void Cropper::cropByPointIndex(CloudXYZ::ConstPtr &inputCloud,
                               CloudPC::Ptr &inputPrincipalCurvatures, int pointIndex,
                               std::string radiusOrK, float radiusOrKvalue,
                               CloudXYZ::Ptr &outputCloud,
                               CloudPC::Ptr &outputPrincipalCurvaturesCloud,
                               std::vector<int> &pointIndexSearch, std::vector<float> &pointSquaredDistance)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(inputCloud);

    if (radiusOrK == "radius")
    {
        if (kdTree.radiusSearch(inputCloud->points[pointIndex], radiusOrKvalue, pointIndexSearch, pointSquaredDistance) > 0)
        {
            for (int j = 0; j < pointIndexSearch.size(); j++)
            {
                outputCloud->points.push_back(inputCloud->points[pointIndexSearch[j]]);
                outputPrincipalCurvaturesCloud->points.push_back(inputPrincipalCurvatures->points[pointIndexSearch[j]]);
            }
        }
    }
    else
    {
        if (radiusOrK == "k")
        {
            if (kdTree.nearestKSearch(inputCloud->points[pointIndex], radiusOrKvalue, pointIndexSearch, pointSquaredDistance) > 0)
            {
                for (int j = 0; j < pointIndexSearch.size(); j++)
                {
                    outputCloud->points.push_back(inputCloud->points[pointIndexSearch[j]]);
                    outputPrincipalCurvaturesCloud->points.push_back(inputPrincipalCurvatures->points[pointIndexSearch[j]]);
                }
            }
        }
        else
        {
            throw std::runtime_error("Use only 'radius' or 'k' in cropByPointIndex");
            return;
        }
    }
}

void Cropper::cropByPointValues(CloudXYZ::Ptr &inputCloud, float xValue, float yValue, float zValue,
                                std::string radiusOrK, float radiusOrKvalue,
                                CloudXYZ::Ptr &outputCloud)
{
    std::vector<int> defaultPointIndexSearch;
    std::vector<float> defaultPointSquaredDistance;

    CloudPC::Ptr defaultPrincipalCurvaturesCloud(new CloudPC());
    defaultPrincipalCurvaturesCloud->points.resize(inputCloud->points.size());

    defaultPointIndexSearch.resize(inputCloud->points.size());
    defaultPointSquaredDistance.resize(inputCloud->points.size());

    Cropper::cropByPointValues(inputCloud, defaultPrincipalCurvaturesCloud, xValue, yValue, zValue, radiusOrK,
                               radiusOrKvalue, outputCloud, defaultPrincipalCurvaturesCloud, defaultPointIndexSearch, defaultPointSquaredDistance);
}

void Cropper::cropByPointValues(CloudXYZ::Ptr &inputCloud,
                                CloudPC::Ptr &inputPrincipalCurvatures, float xValue,
                                float yValue, float zValue, std::string radiusOrK, float radiusOrKvalue,
                                CloudXYZ::Ptr &outputCloud,
                                CloudPC::Ptr &outputPrincipalCurvaturesCloud,
                                std::vector<int> &pointIndexSearch, std::vector<float> &pointSquaredDistance)
{
    CloudXYZ::ConstPtr constCloud = inputCloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
    kdTree.setInputCloud(constCloud);

    int pointIndex = -1;

    for (int i = 0; i < constCloud->points.size(); i++)
    {
        if (constCloud->points[i].x == xValue && constCloud->points[i].y == yValue && constCloud->points[i].z == zValue)
        {
            pointIndex = i;
            break;
        }
    }

    if (pointIndex == -1)
    {
        throw std::runtime_error("This point doesn't exist on Input Cloud, please insert on it.");
        return;
    }

    if (radiusOrK == "radius")
    {
        if (kdTree.radiusSearch(constCloud->points[pointIndex], radiusOrKvalue, pointIndexSearch, pointSquaredDistance) > 0)
        {
            for (int j = 0; j < pointIndexSearch.size(); j++)
            {
                outputCloud->points.push_back(constCloud->points[pointIndexSearch[j]]);
                outputPrincipalCurvaturesCloud->points.push_back(inputPrincipalCurvatures->points[pointIndexSearch[j]]);
            }
        }
    }
    else
    {
        if (radiusOrK == "k")
        {
            if (kdTree.nearestKSearch(constCloud->points[pointIndex], radiusOrKvalue, pointIndexSearch, pointSquaredDistance) > 0)
            {
                for (int j = 0; j < pointIndexSearch.size(); j++)
                {
                    outputCloud->points.push_back(constCloud->points[pointIndexSearch[j]]);
                    outputPrincipalCurvaturesCloud->points.push_back(inputPrincipalCurvatures->points[pointIndexSearch[j]]);
                }
            }
        }
        else
        {
            throw std::runtime_error("Use only 'radius' or 'k' in cropByPointIndex");
            return;
        }
    }
}

void Cropper::removeIsolatedPoints(CloudXYZ::Ptr &inputCloud, float radiusSearchSize,
                                   int pointsThreshold, CloudXYZ::Ptr &outputCloud)
{
    CloudPC::Ptr defaultPrincipalCurvaturesCloud;
    defaultPrincipalCurvaturesCloud->points.resize(inputCloud->points.size());

    Cropper::removeIsolatedPoints(inputCloud, defaultPrincipalCurvaturesCloud, radiusSearchSize, pointsThreshold, outputCloud, defaultPrincipalCurvaturesCloud);
}

void Cropper::removeIsolatedPoints(CloudXYZ::Ptr &inputCloud,
                                   CloudPC::Ptr &inputPrincipalCurvatures,
                                   float radiusSearchSize, int pointsThreshold,
                                   CloudXYZ::Ptr &outputCloud,
                                   CloudPC::Ptr &outputPrincipalCurvaturesCloud)
{
    for (int i = 0; i < inputCloud->points.size(); i++)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
        kdTree.setInputCloud(inputCloud);

        std::vector<int> pointIndexRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (kdTree.radiusSearch(inputCloud->points[i], radiusSearchSize, pointIndexRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            if (pointIndexRadiusSearch.size() >= pointsThreshold)
            {
                outputCloud->points.push_back(inputCloud->points[i]);
                outputPrincipalCurvaturesCloud->points.push_back(inputPrincipalCurvatures->points[i]);
            }
        }

        pointIndexRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
    }
}
