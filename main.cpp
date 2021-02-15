//
// Created by marcusv on 12/03/19.
//
#include <iostream>
#include <vector>
#include <cmath>
#include <string.h>
#include <fstream>
#include <chrono>

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
#include "Utils.h"

int main(int, char **argv)
{
    try
    {
        auto totalStart = std::chrono::steady_clock::now();
        CloudsLog cloudsLog;

        typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
        typedef pcl::PointCloud<pcl::Normal> CloudNormal;
        typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

        std::string filename = argv[1];

        //Declaring clouds
        CloudXYZ::Ptr cloud(new CloudXYZ);

        std::cout << "Reading " << filename << std::endl;
        cloud = Utils::loadCloudFile(filename);

        cloudsLog.add("0. Loaded Cloud from PCD", cloud);

        CloudXYZ::Ptr filteredCloud(new CloudXYZ);

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
        std::vector<int> notNaNIndicesOfShapeIndexes;
        std::vector<float> shapeIndexes;

        // Computating Shape Indexes
        Computation::shapeIndexComputation(principalCurvaturesCloud, shapeIndexes, notNaNIndicesOfShapeIndexes);
        if (notNaNIndicesOfShapeIndexes.size() != filteredCloud->points.size() || notNaNIndicesOfShapeIndexes.size() != principalCurvaturesCloud->points.size())
        {
            NosetipFinder::removeNonExistingIndices(filteredCloud, notNaNIndicesOfShapeIndexes);
            NosetipFinder::removeNonExistingIndices(principalCurvaturesCloud, notNaNIndicesOfShapeIndexes);
        }

        // int scaleShapeIndexMinThreshold = -1;
        // int scaleShapeIndexMaxThreshold = 1;
        // NosetipFinder::scaleShapeIndexes(shapeIndexes, scaleShapeIndexMinThreshold, scaleShapeIndexMaxThreshold);
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
        double gaussianCurvatureLimit = 0.015; //0.015
        double largestShapeIndexLimit = -0.75; // -0.7

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

        //When flexing parameters, at some point, we try to get the average Z of cloud: (smallerZ + largestZ) / 2
        //This is a control variable to check if we already do this and move to the next try
        bool triedToGetAverageZ = false;

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
                    gaussianCurvatureLimit = gaussianCurvatureLimit - 0.001;
                }
                else if (largestShapeIndexLimit < 1)
                {
                    largestShapeIndexLimit = largestShapeIndexLimit + 0.1;
                }
                else
                {
                    // largestShapeIndexLimit is at max 1. If even in this max there is not enough points, flexibilize gaussian curvature until find it.
                    gaussianCurvatureLimit = gaussianCurvatureLimit - 0.001;
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

                if (gaussianCurvatureLimit > 0.008)
                {
                    gaussianCurvatureLimit = gaussianCurvatureLimit - 0.001;
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
                            if (cropSize == 100 && !triedToGetAverageZ)
                            {
                                pointToCropFrom.z = (largestZ + smallerZ) / 2;
                                triedToGetAverageZ = true;
                            }
                            else
                            {
                                if (cropSize < 240)
                                {
                                    cropSize += 20;
                                }
                                else
                                {
                                    if (gaussianCurvatureLimit > 0.000)
                                    {
                                        gaussianCurvatureLimit = gaussianCurvatureLimit - 0.001;
                                    }
                                    else
                                    {

                                        if (croppedSIandGC->points.empty())
                                        {
                                            throw std::runtime_error("Failed. croppedSIandGC is empty");
                                        }
                                        continueLoop = false;
                                    }
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
        // std::cout << "20" << std::endl;
        cloudsLog.add("5. Cloud after removing isolated points", cloudFinal);

        pcl::PointXYZ noseTip;
        int chooseANoseTipThreshold = 15;
        if (cloudFinal->points.size() < 15)
        {
            chooseANoseTipThreshold = cloudFinal->points.size();
        }
        noseTip = NosetipFinder::chooseANoseTip(cloudFinal, cloudPCFinal, chooseANoseTipThreshold);
        std::cout << noseTip << " choosed as nose tip!" << std::endl;

        CloudXYZ::Ptr noseTipCloud(new CloudXYZ);
        noseTipCloud->points.push_back(noseTip);
        cloudsLog.add("6. Found nosetip", noseTipCloud);

        std::vector<CloudsLogEntry> cloudsLogEntries = cloudsLog.getLogs();
        std::cout << "Amount of clouds in log: " << cloudsLogEntries.size() << std::endl;

        auto totalEnd = std::chrono::steady_clock::now();
        auto totalDiff = totalEnd - totalStart;

        if (argv[2])
        {
            NosetipFinder::saveNoseTip(noseTip, argv[2], argv[1]);
        }

        if (strcmp(argv[3], "visualizar") == 0)
        {
            pcl::visualization::PCLVisualizer viewer;
            for (int k = 0; k < cloudsLogEntries.size(); k++)
            {
                int r = rand() % 256;
                int g = rand() % 256;
                int b = rand() % 256;
                int pointsSize = 2;

                // Highlighting the found nosetip
                if (k == cloudsLogEntries.size() - 1)
                {
                    r = 255;
                    g = 255;
                    b = 255;
                    pointsSize = 10;
                    std::cout << cloudsLogEntries[k].cloudLabel << std::endl;
                }

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloudColor(cloudsLogEntries[k].cloud, r, g, b);
                viewer.addPointCloud<pcl::PointXYZ>(cloudsLogEntries[k].cloud, cloudColor, "cloudlog-" + std::to_string(k), 0);
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointsSize, "cloudlog-" + std::to_string(k));
            }

            while (!viewer.wasStopped())
            {
                viewer.spinOnce(100);
            }
        }

        if (argv[4] && argv[5] && argv[6])
        {
            std::cout << "Reading " << argv[4] << " to verify nosetip." << std::endl;

            CloudXYZ::Ptr verificationCloud(new CloudXYZ);

            if (pcl::io::loadPCDFile(argv[4], *verificationCloud) == -1)
            {
                throw std::runtime_error("Couldn't read verification file");
                return (-1);
            }

            bool isAGoodNoseTip;

            if (
                NosetipFinder::itsAGoodNoseTip(noseTip, verificationCloud->points[0].x, verificationCloud->points[0].y, verificationCloud->points[0].z, 20))
            {
                std::cout << "It's a good nose tip" << std::endl;
                isAGoodNoseTip = true;
            }
            else
            {
                std::cout << "It is not a good nose tip" << std::endl;
                isAGoodNoseTip = false;
            }

            Utils::saveProcessingResult(
                argv[5],
                argv[1],
                isAGoodNoseTip,
                std::chrono::duration<double, std::milli>(totalDiff).count(),
                verificationCloud->points[0],
                noseTip);
        }
        else if (strcmp(argv[3], "visualizar") == 0)
        {
            std::string resp;
            std::cout << "It's a good nose tip? Y/N" << std::endl;
            std::cin >> resp;

            bool boolResp = false;
            if (resp == "y" || resp == "Y")
            {
                boolResp = true;
            }

            Utils::saveProcessingResult(
                argv[4],
                argv[1],
                boolResp,
                std::chrono::duration<double, std::milli>(totalDiff).count(),
                noseTip);
        }
    }
    catch (std::exception &e)
    {
        std::string errorCause = e.what();
        std::cout << errorCause << std::endl;
        Utils::saveErrorResult(argv[6], argv[1], errorCause);
    }
}
