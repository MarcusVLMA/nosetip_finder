//
// Created by marcusv on 10/03/19.
//

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include "Computation.h"

void Computation::normalComputation(CloudXYZ::Ptr inputCloud, std::string radiusOrKSearch,
                                    float radiusOrK, CloudNormal::Ptr outputNormalCloud)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    normalEstimation.setInputCloud(inputCloud);
    normalEstimation.setSearchMethod(tree);

    if (radiusOrKSearch == "radius")
    {
        normalEstimation.setRadiusSearch(radiusOrK);
    }
    else
    {
        if (radiusOrKSearch == "k")
        {
            normalEstimation.setKSearch(radiusOrK);
        }
        else
        {
            PCL_ERROR("Use 'radius' or 'k' in normalComputation");
        }
    }

    normalEstimation.compute(*outputNormalCloud);
}

void Computation::principalCurvaturesComputation(CloudXYZ::Ptr inputCloud,
                                                 CloudNormal::Ptr normalInputCloud,
                                                 std::string radiusOrKSearch, int radiusOrK,
                                                 CloudPC::Ptr
                                                     outputPrincipalCurvaturesCloud)
{
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

    principalCurvaturesEstimation.setInputCloud(inputCloud);
    principalCurvaturesEstimation.setInputNormals(normalInputCloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    principalCurvaturesEstimation.setSearchMethod(tree);

    if (radiusOrKSearch == "radius")
    {
        principalCurvaturesEstimation.setRadiusSearch(radiusOrK);
    }
    else
    {
        if (radiusOrKSearch == "k")
        {
            principalCurvaturesEstimation.setKSearch(radiusOrK);
        }
        else
        {
            PCL_ERROR("Use 'radius' or 'k' in principalCurvaturesComputation");
        }
    }

    principalCurvaturesEstimation.compute(*outputPrincipalCurvaturesCloud);
}

void Computation::shapeIndexComputation(CloudPC::Ptr principalCurvaturesCloud,
                                        std::vector<float> &outputShapeIndexes, std::vector<int> &nanIndices)
{
    float shapeIndex;
    float k1;
    float k2;
    float atg;

    for (int i = 0; i < principalCurvaturesCloud->points.size(); i++)
    {
        k1 = principalCurvaturesCloud->points[i].pc1;
        k2 = principalCurvaturesCloud->points[i].pc2;

        if (k1 >= k2)
        {
            atg = atan((k2 + k1) / (k2 - k1));
            shapeIndex = (2 / M_PI) * (atg);
        }
        else
        {
            atg = atan((k1 + k2) / (k1 - k2));
            shapeIndex = (2 / M_PI) * (atg);
        }

        if (!std::isnan(shapeIndex))
        {
            outputShapeIndexes.push_back(shapeIndex);
        }
        else
        {
            nanIndices.push_back(i);
        }
    }
}

float Computation::findMaxValueInPointCloud(CloudXYZ::Ptr inputCloud, char axis)
{
    float maxValue;

    if (axis == 'x')
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (i != 0)
            {
                if (maxValue < inputCloud->points[i].x)
                {
                    maxValue = inputCloud->points[i].x;
                }
            }
            else
            {
                maxValue = inputCloud->points[i].x;
            }
        }

        return maxValue;
    }
    else
    {
        if (axis == 'y')
        {
            for (int i = 0; i < inputCloud->points.size(); i++)
            {
                if (i != 0)
                {
                    if (maxValue < inputCloud->points[i].y)
                    {
                        maxValue = inputCloud->points[i].y;
                    }
                }
                else
                {
                    maxValue = inputCloud->points[i].y;
                }
            }

            return maxValue;
        }
        else
        {
            if (axis == 'z')
            {
                for (int i = 0; i < inputCloud->points.size(); i++)
                {
                    if (i != 0)
                    {
                        if (maxValue < inputCloud->points[i].z)
                        {
                            maxValue = inputCloud->points[i].z;
                        }
                    }
                    else
                    {
                        maxValue = inputCloud->points[i].z;
                    }
                }

                return maxValue;
            }
            else
            {
                PCL_ERROR("Use 'x', 'y' or 'z' in findMaxValueInPointCloud");
            }
        }
    }

    return NULL;
}

float Computation::findMinValueInPointCloud(CloudXYZ::Ptr inputCloud, char axis)
{
    float minValue;

    if (axis == 'x')
    {
        for (int i = 0; i < inputCloud->points.size(); i++)
        {
            if (i != 0)
            {
                if (minValue > inputCloud->points[i].x)
                {
                    minValue = inputCloud->points[i].x;
                }
            }
            else
            {
                minValue = inputCloud->points[i].x;
            }
        }

        return minValue;
    }
    else
    {
        if (axis == 'y')
        {
            for (int i = 0; i < inputCloud->points.size(); i++)
            {
                if (i != 0)
                {
                    if (minValue > inputCloud->points[i].y)
                    {
                        minValue = inputCloud->points[i].y;
                    }
                }
                else
                {
                    minValue = inputCloud->points[i].y;
                }
            }

            return minValue;
        }
        else
        {
            if (axis == 'z')
            {
                for (int i = 0; i < inputCloud->points.size(); i++)
                {
                    if (i != 0)
                    {
                        if (minValue < inputCloud->points[i].z)
                        {
                            minValue = inputCloud->points[i].z;
                        }
                    }
                    else
                    {
                        minValue = inputCloud->points[i].z;
                    }
                }

                return minValue;
            }
            else
            {
                PCL_ERROR("Use 'x', 'y' or 'z' in findMaxValueInPointCloud");
            }
        }
    }

    return NULL;
}

std::vector<int> Computation::findKPointsWithLargestGaussianCurvatures(CloudXYZ &inputCloud,
                                                                       CloudPC
                                                                           &inputPrincipalCurvaturesCloud,
                                                                       int kPoints)
{
    std::vector<int> largestGaussianCurvaturesIndices;

    return std::vector<int>();
}
