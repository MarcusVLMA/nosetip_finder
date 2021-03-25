//
// Created by marcusv on 17/02/21.
//
#include <iostream>
#include <vector>
#include <cmath>
#include <string.h>
#include <fstream>
#include <chrono>
#include <sstream>

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
#include "Computation.h"
#include "Cropper.h"
#include "NosetipFinder.h"
#include "CloudsLog.h"
#include "Utils.h"

// A good value for gaussianCurvatureLimit is 0.015
// A good value for largestShapeIndexLimit is -0.85
MainResponse Main::run(
    std::string filename,
    bool flexibilizeThresholds,
    bool flexibilizeCrop,
    int computationRadiusOrKSize,
    std::string computationMethod,
    double minGaussianCurvature,
    double shapeIndexLimit,
    float minCropSize,
    float maxCropSize,
    int minPointsToContinue,
    float removeIsolatedPointsRadius,
    int removeIsolatedPointsThreshold,
    int nosetipSearchRadius,
    int pointIndexToAnalyze)
{
  auto totalStart = std::chrono::steady_clock::now();
  CloudsLog cloudsLog;

  typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
  typedef pcl::PointCloud<pcl::Normal> CloudNormal;
  typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

  //Declaring clouds
  CloudXYZ::Ptr cloud(new CloudXYZ);

  std::cout << "Reading " << filename << std::endl;
  cloud = Utils::loadCloudFile(filename);

  pcl::PointXYZ *pointToAnalyze{0};

  if (pointIndexToAnalyze >= 0)
  {
    pcl::PointXYZ p = cloud->points[pointIndexToAnalyze];
    pointToAnalyze = &p;
    *pointToAnalyze = p;
  }

  CloudXYZ::Ptr filteredCloud(new CloudXYZ);

  //Removing NaNs
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);

  //Declaring Normal Clouds
  CloudNormal::Ptr normalCloud(new CloudNormal);
  CloudNormal::Ptr filteredNormalCloud(new CloudNormal);

  //Computating the normals
  Computation::normalComputation(filteredCloud, computationMethod, computationRadiusOrKSize, normalCloud);

  //We didn't use 'indices' vector, clearing it to re-use the variable.
  indices.clear();

  //Removing NaNs from Normal Cloud
  pcl::removeNaNNormalsFromPointCloud(*normalCloud, *filteredNormalCloud, indices);

  /*
    * After removing NaNs from Normal Cloud, maybe the size of the XYZ Cloud isn't the same size of Normal Cloud,
    * so we keep in the XYZ Cloud only the indices which are present in Normal Cloud.
    */
  NosetipFinder::removeNonExistingIndices(filteredCloud, indices);

  //Declaring Principal Curvatures Cloud
  CloudPC::Ptr principalCurvaturesCloud(new CloudPC);

  //Computating Principal Curvatures
  Computation::principalCurvaturesComputation(filteredCloud, filteredNormalCloud, computationMethod, computationRadiusOrKSize, principalCurvaturesCloud);
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

  //Iterating to find a good crop of points

  //When flexing parameters, at some point, we try to get the average Z of cloud: (smallerZ + largestZ) / 2
  //This is a control variable to check if we already do this and move to the next try
  bool triedToGetAverageZ = false;
  float cropSize = minCropSize;
  double gaussianCurvatureLimit = minGaussianCurvature;
  double largestShapeIndexLimit = shapeIndexLimit;

  int gcAndSIFilterCount = 1;
  int cropCount = 1;
  while (continueLoop)
  {
    //Thresholding points by Shape Indexes and Gaussian Curvatures
    NosetipFinder::thresholdByShapeIndexAndGaussianCurvature(
        filteredCloud, shapeIndexes, principalCurvaturesCloud, -1, largestShapeIndexLimit, gaussianCurvatureLimit,
        cloudAfterSIandGCfilter, shapeIndexAfterSIandGCfilter, pcCloudAfterSIandGCfilter);

    std::string logLabel;
    std::stringstream siStream, gcStream;
    siStream << std::fixed << std::setprecision(2) << largestShapeIndexLimit;
    gcStream << std::fixed << std::setprecision(3) << gaussianCurvatureLimit;
    logLabel = "1." + std::to_string(gcAndSIFilterCount) + " SI(" + siStream.str() + ") and GC(" + gcStream.str() + ") filter";
    cloudsLog.add(logLabel, cloudAfterSIandGCfilter);
    gcAndSIFilterCount++;

    //Checking if there is enough points to continue the algorithm
    if (cloudAfterSIandGCfilter->points.size() < minPointsToContinue)
    {
      if (!flexibilizeThresholds)
      {
        throw std::runtime_error("After Shape Index and Gaussian Curvature filter, cloud doesn't have the minimum size to continue. You can set a smaller 'minPointsToContinue' or set 'flexibilizeThresholds' as 'true'.");
      }

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

    //Cropping the cloud using the point calculated early. The result cloud is in 'croppedSIandGC'
    Cropper::cropByPointValues(
        cloudAfterSIandGCfilter, pcCloudAfterSIandGCfilter, pointToCropFrom.x, pointToCropFrom.y, pointToCropFrom.z,
        "radius", cropSize, croppedSIandGC, croppedPCSIandGC, indexSearch, squaredDistanceSearch);

    std::stringstream cropStream;
    cropStream << std::fixed << std::setprecision(0) << cropSize;

    std::stringstream cropPointXStream;
    std::stringstream cropPointYStream;
    std::stringstream cropPointZStream;
    cropPointXStream << std::fixed << std::setprecision(2) << pointToCropFrom.x;
    cropPointYStream << std::fixed << std::setprecision(2) << pointToCropFrom.y;
    cropPointZStream << std::fixed << std::setprecision(2) << pointToCropFrom.z;
    logLabel = "2." + std::to_string(cropCount) + " Crop(" + cropStream.str() + "mm) at (" + cropPointXStream.str() + "," + cropPointYStream.str() + "," + cropPointZStream.str() + ")";
    cloudsLog.add(logLabel, croppedSIandGC);
    cropCount++;

    //Checking if there if enough points to continue
    if (croppedSIandGC->points.size() > minPointsToContinue)
    {
      continueLoop = false;
    }
    else
    {
      if (!flexibilizeThresholds && !flexibilizeCrop)
      {
        throw std::runtime_error("After Crop, cloud doesn't have the minimum size to continue. You can set a smaller 'minPointsToContinue' or set 'flexibilizeThresholds' and/or 'flexibilizeCrop' as 'true'.");
      }

      croppedPCSIandGC->points.clear();
      croppedSIandGC->points.clear();

      cloudAfterSIandGCfilter->points.clear();
      pcCloudAfterSIandGCfilter->points.clear();

      indexSearch.clear();
      squaredDistanceSearch.clear();

      if (gaussianCurvatureLimit > 0.008 && flexibilizeThresholds)
      {
        gaussianCurvatureLimit = gaussianCurvatureLimit - 0.001;
      }
      else
      {
        if (largestShapeIndexLimit < 1 && flexibilizeThresholds)
        {
          largestShapeIndexLimit = largestShapeIndexLimit + 0.1;
        }
        else
        {
          if (flexibilizeCrop)
          {
            pointToCropFrom.z = pointToCropFrom.z - 2;

            if (pointToCropFrom.z < smallerZ)
            {
              if (cropSize == minCropSize && !triedToGetAverageZ)
              {
                pointToCropFrom.z = (largestZ + smallerZ) / 2;
                triedToGetAverageZ = true;
              }
              else
              {
                if (cropSize < maxCropSize)
                {
                  cropSize += 20;
                }
              }
            }
          }

          if (((flexibilizeCrop && cropSize >= maxCropSize) || (!flexibilizeCrop && gaussianCurvatureLimit > 0.000)) && flexibilizeThresholds)
          {
            gaussianCurvatureLimit = gaussianCurvatureLimit - 0.001;
          }
          else if (gaussianCurvatureLimit <= 0.000)
          {
            continueLoop = false;
          }
        }
      }
    }
  }

  CloudXYZ::Ptr cloudFinal(new CloudXYZ);
  CloudPC::Ptr cloudPCFinal(new CloudPC);

  std::vector<float> shapeIndexFinal;
  Cropper::removeIsolatedPoints(croppedSIandGC, shapeIndexes, removeIsolatedPointsRadius, removeIsolatedPointsThreshold, flexibilizeThresholds, minPointsToContinue, cloudFinal, shapeIndexFinal, cloudsLog);

  pcl::PointXYZ noseTip;
  int chooseANoseTipThreshold = minPointsToContinue;
  if (cloudFinal->points.size() < minPointsToContinue && flexibilizeThresholds)
  {
    chooseANoseTipThreshold = cloudFinal->points.size();
  }

  noseTip = NosetipFinder::chooseANoseTip(cloudFinal, nosetipSearchRadius, cloudsLog);
  std::cout << noseTip << " choosed as nose tip!" << std::endl;

  CloudXYZ::Ptr noseTipCloud(new CloudXYZ);
  noseTipCloud->points.push_back(noseTip);

  std::string noseTipLabel = "5. Nosetip (Searched in " + std::to_string(chooseANoseTipThreshold) + " points)";

  std::vector<CloudsLogEntry> cloudsLogEntries = cloudsLog.getLogs();
  std::cout << "Amount of clouds in log: " << cloudsLogEntries.size() << std::endl;

  auto totalEnd = std::chrono::steady_clock::now();
  auto totalDiff = totalEnd - totalStart;

  struct MainResponse response;
  response.cloudsLog = cloudsLog;
  response.noseTip = noseTip;
  response.executionTime = std::chrono::duration<double, std::milli>(totalDiff).count();

  struct PointAnalysis pa;
  pa.isEmpty = true;

  // If enters here, pa.isEmpty will be set to false.
  if (pointIndexToAnalyze >= 0)
  {
    pa = Utils::getPointAnalysis(*pointToAnalyze, filteredCloud, filteredNormalCloud, principalCurvaturesCloud, shapeIndexes);
  }

  response.pointAnalysis = pa;

  return response;
}