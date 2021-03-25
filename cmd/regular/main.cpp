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
#include "../../src/Main.h"
#include "../../src/Utils.h"
#include "../../src/NosetipFinder.h"

int main(int, char **argv)
{
    try
    {
        std::string filename = argv[1];

        std::string trueString = "true";

        bool flexibilizeThresholds = false;
        if (trueString.compare(argv[2]) == 0)
        {
            flexibilizeThresholds = true;
        }

        bool flexibilizeCrop = false;
        if (trueString.compare(argv[3]) == 0)
        {
            flexibilizeCrop = true;
        }
        int computationRadiusOrKSize = std::stoi(argv[4]);
        std::string computationMethod = argv[5];
        double minGaussianCurvature = std::stod(argv[6]);
        double shapeIndexLimit = std::stod(argv[7]);
        float minCropSize = std::stof(argv[8]);
        float maxCropSize = std::stof(argv[9]);
        int minPointsToContinue = std::stoi(argv[10]);
        float removeIsolatedPointsRadius = std::stof(argv[11]);
        int removeIsolatedPointsThreshold = std::stoi(argv[12]);
        int nosetipSearchRadius = std::stoi(argv[13]);

        struct MainResponse response = Main::run(
            filename,
            flexibilizeThresholds,
            flexibilizeCrop,
            computationRadiusOrKSize,
            computationMethod,
            minGaussianCurvature,
            shapeIndexLimit,
            minCropSize,
            maxCropSize,
            minPointsToContinue,
            removeIsolatedPointsRadius,
            removeIsolatedPointsThreshold,
            nosetipSearchRadius,
            -1); // Not using analysis of point for now

        pcl::PointXYZ noseTip = response.noseTip;
        std::vector<CloudsLogEntry> cloudsLogEntries = response.cloudsLog.getLogs();
        double executionTime = response.executionTime;

        if (argv[14])
        {
            NosetipFinder::saveNoseTip(noseTip, argv[14], filename);
        }

        if (strcmp(argv[15], "visualizar") == 0)
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

        if (argv[16] && argv[17] && argv[18])
        {
            std::cout << "Reading " << argv[16] << " to verify nosetip." << std::endl;

            CloudXYZ::Ptr verificationCloud(new CloudXYZ);

            if (pcl::io::loadPCDFile(argv[16], *verificationCloud) == -1)
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
                argv[17],
                filename,
                isAGoodNoseTip,
                executionTime,
                verificationCloud->points[0],
                noseTip);
        }
        else if (strcmp(argv[15], "visualizar") == 0)
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
                argv[16],
                filename,
                boolResp,
                executionTime,
                noseTip);
        }
    }
    catch (std::exception &e)
    {
        std::string errorCause = e.what();
        std::cout << errorCause << std::endl;

        std::string errorFile;
        if (argv[18])
        {
            errorFile = argv[18];
        }
        else
        {
            errorFile = argv[17];
        }

        Utils::saveErrorResult(errorFile, argv[1], errorCause);
    }
}
