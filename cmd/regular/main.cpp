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
#include "Main.h"
#include "Utils.h"
#include "NosetipFinder.h"

int main(int, char **argv)
{
    try
    {
        struct MainResponse response = Main::run(argv[1]);

        pcl::PointXYZ noseTip = response.noseTip;
        std::vector<CloudsLogEntry> cloudsLogEntries = response.cloudsLog.getLogs();
        double executionTime = response.executionTime;

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
                executionTime,
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
                executionTime,
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
