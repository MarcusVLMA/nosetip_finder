//
// Created by marcusv on 5/02/21.
//

#ifndef NOSETIP_FINDER_UTILS_H
#define NOSETIP_FINDER_UTILS_H

#include <string.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
typedef pcl::PointCloud<pcl::Normal> CloudNormal;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

struct PointAnalysis
{
    pcl::Normal normal;
    pcl::PrincipalCurvatures principalCurvatures;
    float shapeIndex;
    float gaussianCurvature;
    bool isEmpty;
};

class Utils
{
public:
    Utils(){};
    ~Utils(){};

    CloudXYZ::Ptr static loadCloudFile(std::string filename);

    PointAnalysis static getPointAnalysis(
        pcl::PointXYZ point,
        CloudXYZ::Ptr &inputCloud,
        CloudNormal::Ptr &normalCloud,
        CloudPC::Ptr &principalCurvaturesCloud,
        std::vector<float> shapeIndexes);

    void static saveProcessingResult(
        std::string outputFilename,
        std::string inputCloudFilename,
        bool isAGoodNoseTip,
        double totalMilliseconds,
        pcl::PointXYZ originalNoseTip,
        pcl::PointXYZ foundNoseTip);

    void static saveProcessingResult(
        std::string outputFilename,
        std::string inputCloudFilename,
        bool isAGoodNoseTip,
        double totalMilliseconds,
        pcl::PointXYZ foundNoseTip);

    void static saveErrorResult(
        std::string outputFilename,
        std::string inputCloudFilename,
        std::string error);
};

#endif