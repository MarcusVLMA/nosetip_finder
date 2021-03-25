//
// Created by marcusv on 10/03/19.
//

#ifndef NOSETIP_FINDER_NOSETIPFINDER_H
#define NOSETIP_FINDER_NOSETIPFINDER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include "CloudsLog.h"

class NosetipFinder
{
    typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
    typedef pcl::PointCloud<pcl::Normal> CloudNormal;
    typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

public:
    void static removeNonExistingIndices(CloudXYZ::Ptr &inputCloud, std::vector<int> indicesToKeep);

    void static removeNonExistingIndices(CloudPC::Ptr &inputCloud, std::vector<int> indicesToKeep);

    void static thresholdByShapeIndex(
        CloudXYZ::Ptr &inputCloud,
        std::vector<float> shapeIndexes,
        float thresholdMin,
        float thresholdMax,
        CloudXYZ::Ptr &outputCloud,
        std::vector<float> outputShapeIndexes);

    void static thresholdByGaussianCurvature(
        CloudXYZ::Ptr &inputCloud,
        CloudPC::Ptr &inputPrincipalCurvaturesCloud,
        float thresholdMin,
        CloudXYZ::Ptr &outputCloud,
        CloudPC::Ptr &outputPrincipalCurvaturesCloud);

    void static thresholdByShapeIndexAndGaussianCurvature(
        CloudXYZ::Ptr &inputCloud,
        std::vector<float> shapeIndexes,
        CloudPC::Ptr &inputPrincipalCurvaturesCloud,
        float thresholdShapeIndexMin, float thresholdShapeIndexMax, float thresholdPrincipalCurvatureMin,
        CloudXYZ::Ptr &outputCloud,
        std::vector<float> outputShapeIndexes,
        CloudPC::Ptr &outputPrincipalCurvaturesCloud);

    pcl::PointXYZ static chooseANoseTip(
        CloudXYZ::Ptr inputCloud,
        int searchRadius,
        CloudsLog &cloudsLog);

    bool static itsAGoodNoseTip(pcl::PointXYZ noseTip, float xValue, float yValue, float zValue, float maxDistance);

    bool static saveNoseTip(pcl::PointXYZ noseTip, std::string filename, std::string cloudName);

    int static findNosetip(int argc, char **argv);
};

#endif //NOSETIP_FINDER_NOSETIPFINDER_H
