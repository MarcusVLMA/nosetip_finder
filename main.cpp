//
// Created by marcusv on 12/03/19.
//
#include <iostream>
#include <vector>
#include <cmath>
#include <string.h>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include "Computation.h"
#include "Cropper.h"
#include "NosetipFinder.h"
#include "CloudsLog.h"

int main(int, char **argv)
{
    CloudsLog cloudsLog;

    typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
    typedef pcl::PointCloud<pcl::Normal> CloudNormal;
    typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

    std::string filename = argv[1];

    //Declaring clouds
    CloudXYZ::Ptr cloud(new CloudXYZ);
    CloudXYZ::Ptr filteredCloud(new CloudXYZ);
    std::cout << "Reading " << filename << std::endl;
    //Loading PCD file to cloud
    if (pcl::io::loadPCDFile(filename, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file");
        return (-1);
    }

    cloudsLog.add("0. Loaded Cloud from PCD", cloud);

    //Removing NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);

    cloudsLog.add("1. Loaded Cloud filtered from NaNs", filteredCloud);

    //Declaring Normal Clouds
    CloudNormal::Ptr normalCloud(new CloudNormal);
    CloudNormal::Ptr filteredNormalCloud(new CloudNormal);

    //Computating the normals
    Computation::normalComputation(filteredCloud, "radius", 13, normalCloud);

    //We didn't use 'indices' vector, clearing it to re-use the variable.
    indices.clear();

    //Removing NaNs from Normal Cloud
    pcl::removeNaNNormalsFromPointCloud(*normalCloud, *filteredNormalCloud, indices);
    /*
     * After removing NaNs from Normal Cloud, maybe the size of the XYZ Cloud isn't the same size of Normal Cloud,
     * so we keep in the XYZ Cloud only the indices which are present in Normal Cloud.
     */
    NosetipFinder::removeNonExistingIndices(filteredCloud, indices);

    cloudsLog.add("2. Cloud after removing indices not present on Normal cloud", filteredCloud);

    //Declaring Principal Curvatures Cloud
    CloudPC::Ptr principalCurvaturesCloud(new CloudPC);

    //Computating Principal Curvatures
    Computation::principalCurvaturesComputation(filteredCloud, filteredNormalCloud, "radius", 13, principalCurvaturesCloud);
    std::vector<int> nanIndicesOfShapeIndexes;
    std::vector<float> shapeIndexes;

    //Computating Shape Indexes
    Computation::shapeIndexComputation(principalCurvaturesCloud, shapeIndexes, nanIndicesOfShapeIndexes);
    NosetipFinder::scaleShapeIndexes(shapeIndexes, -1, 1);
    //Finding smaller and largest point values in the cloud
    float smallerX, smallerY, smallerZ, largestX, largestY, largestZ;

    smallerX = Computation::findMinValueInPointCloud(filteredCloud, 'x');
    smallerY = Computation::findMinValueInPointCloud(filteredCloud, 'y');
    smallerZ = Computation::findMinValueInPointCloud(filteredCloud, 'z');

    largestX = Computation::findMaxValueInPointCloud(filteredCloud, 'x');
    largestY = Computation::findMaxValueInPointCloud(filteredCloud, 'y');
    largestZ = Computation::findMaxValueInPointCloud(filteredCloud, 'z');
    //Point which will be used to realize a crop
    pcl::PointXYZ pointToCropFrom;
    pointToCropFrom.x = (smallerX + largestX) / 2;
    pointToCropFrom.y = (smallerY + largestY) / 2;
    pointToCropFrom.z = largestZ;

    //Variables used to iterate and find a good cropprincipalCurvaturesComputation
    bool continueLoop = true;
    double gaussianCurvatureLimit = 0.015;
    double largestShapeIndexLimit = 0;

    //Principal Curvature and XYZ clouds used to storage points AFTER filtering though Gaussian Curvature and Shape Index
    CloudPC::Ptr pcCloudAfterSIandGCfilter(new CloudPC);
    CloudXYZ::Ptr cloudAfterSIandGCfilter(new CloudXYZ);

    //Vector to storage shape indexes AFTER applying filtering by Shape Indexes and Gaussian Curvature
    std::vector<float> shapeIndexAfterSIandGCfilter;

    //Principal Curvature and XYZ clouds used to storage points AFTER cropping
    CloudXYZ::Ptr croppedSIandGC(new CloudXYZ);
    CloudPC::Ptr croppedPCSIandGC(new CloudPC);

    //Vectors used to storage index and squaredDistance on searching of points
    std::vector<int> indexSearch;
    std::vector<float> squaredDistanceSearch;

    float cropSize = 100;
    //Iterating to find a good crop of points

    int iterationCount = 0;
    while (continueLoop)
    {
        //Thresholding points by Shape Indexes and Gaussian Curvatures
        NosetipFinder::thresholdByShapeIndexAndGaussianCurvature(
            filteredCloud, shapeIndexes, principalCurvaturesCloud, -1, largestShapeIndexLimit, gaussianCurvatureLimit,
            cloudAfterSIandGCfilter, shapeIndexAfterSIandGCfilter, pcCloudAfterSIandGCfilter);
        //Checking if there is enough points to continue the algorithm
        if (cloudAfterSIandGCfilter->points.size() < 15)
        {
            cloudAfterSIandGCfilter->points.clear();
            pcCloudAfterSIandGCfilter->clear();

            if (gaussianCurvatureLimit > 0.008)
            {
                gaussianCurvatureLimit = gaussianCurvatureLimit - 0.002;
            }
            else
            {
                largestShapeIndexLimit = largestShapeIndexLimit + 0.1;
            }
            continue;
        }
        cloudAfterSIandGCfilter->points.push_back(pointToCropFrom);

        std::string logLabel = "3." + std::to_string(iterationCount) + " Cloud after SI and GC filter";
        cloudsLog.add(logLabel, cloudAfterSIandGCfilter);

        //Cropping the cloud using the point calculated early. The result cloud is in 'croppedSIandGC'
        Cropper::cropByPointValues(
            cloudAfterSIandGCfilter, pcCloudAfterSIandGCfilter, pointToCropFrom.x, pointToCropFrom.y, pointToCropFrom.z,
            "radius", cropSize, croppedSIandGC, croppedPCSIandGC, indexSearch, squaredDistanceSearch);

        logLabel = "4." + std::to_string(iterationCount) + " Cloud after Crop";
        cloudsLog.add(logLabel, croppedSIandGC);

        //Checking if there if enough points to continue
        if (croppedSIandGC->points.size() > 15)
        {
            continueLoop = false;
        }
        else
        {
            croppedPCSIandGC->points.clear();
            croppedSIandGC->points.clear();

            cloudAfterSIandGCfilter->points.clear();
            pcCloudAfterSIandGCfilter->points.clear();

            indexSearch.clear();
            squaredDistanceSearch.clear();

            if (gaussianCurvatureLimit > 0.010)
            {
                gaussianCurvatureLimit = gaussianCurvatureLimit - 0.002;
            }
            else
            {
                if (largestShapeIndexLimit < 1)
                {
                    largestShapeIndexLimit = largestShapeIndexLimit + 0.1;
                }
                else
                {
                    pointToCropFrom.z = pointToCropFrom.z - 2;

                    if (pointToCropFrom.z < smallerZ)
                    {
                        if (cropSize == 100)
                        {
                            pointToCropFrom.z = (largestZ + smallerZ) / 2;
                        }
                        else
                        {
                            if (cropSize < 240)
                            {
                                cropSize += 20;
                            }
                            else
                            {
                                if (croppedSIandGC->points.empty())
                                {
                                    std::cout << "Não deu" << std::endl;
                                    return (-1);
                                }
                                continueLoop = false;
                            }
                        }
                    }
                }
            }
        }
    }

    CloudXYZ::Ptr cloudFinal(new CloudXYZ);
    CloudPC::Ptr cloudPCFinal(new CloudPC);

    Cropper::removeIsolatedPoints(croppedSIandGC, croppedPCSIandGC, 5, 6, cloudFinal, cloudPCFinal);
    cloudsLog.add("5. Cloud after removing isolated points", cloudFinal);

    pcl::PointXYZ noseTip;

    noseTip = NosetipFinder::chooseANoseTip(cloudFinal, cloudPCFinal, 15);
    std::cout << noseTip << " choosed as nose tip!" << std::endl;
    CloudXYZ::Ptr noseTipCloud(new CloudXYZ);
    noseTipCloud->points.push_back(noseTip);
    std::cout << "==============================" << std::endl
              << std::endl;

    std::vector<CloudsLogEntry> cloudsLogEntries = cloudsLog.getLogs();
    std::cout << "Amount of clouds in log: " << cloudsLogEntries.size() << std::endl;

    for (int k = 0; k < cloudsLogEntries.size(); k++)
    {
        pcl::visualization::PCLVisualizer viewer;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> initialCloudColor(cloudsLogEntries[k].cloud, 255, 0, 0);
        viewer.addPointCloud<pcl::PointXYZ>(cloudsLogEntries[k].cloud, initialCloudColor, "InitialCloud", 0);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "InitialCloud");

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
        }
    }

    if (argv[2])
    {
        NosetipFinder::saveNoseTip(noseTip, argv[2], argv[1]);
    }

    if (strcmp(argv[3], "visualizar") == 0)
    {
        pcl::visualization::PCLVisualizer viewer;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> initialCloudColor(filteredCloud, 255, 0, 0);
        viewer.addPointCloud<pcl::PointXYZ>(filteredCloud, initialCloudColor, "InitialCloud", 0);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "InitialCloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudColor(cloudFinal, 125, 125, 125);
        viewer.addPointCloud<pcl::PointXYZ>(cloudFinal, cloudColor, "Cloud", 0);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> noseTipColor(noseTipCloud, 0, 255, 0);
        viewer.addPointCloud<pcl::PointXYZ>(noseTipCloud, noseTipColor, "NoseTip", 0);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "NoseTip");

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
        }
    }

    if (argv[4])
    {
        std::cout << "Reading " << argv[3] << " to verify nosetip." << std::endl;

        CloudXYZ::Ptr verifyCloud(new CloudXYZ);

        if (pcl::io::loadPCDFile(argv[3], *verifyCloud) == -1)
        {
            PCL_ERROR("Couldn't read verification file");
            return (-1);
        }

        if (
            NosetipFinder::itsAGoodNoseTip(noseTip, verifyCloud->points[13].x, verifyCloud->points[13].y, verifyCloud->points[13].z, 15))
        {
            std::cout << "It's a good nose tip" << std::endl;
        }
        else
        {
            std::cout << "It is not a good nose tip" << std::endl;
        }
    }
}
