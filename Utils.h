//
// Created by marcusv on 5/02/21.
//

#ifndef NOSETIP_FINDER_UTILS_H
#define NOSETIP_FINDER_UTILS_H

#include <string.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Utils
{
public:
  Utils(){};
  ~Utils(){};

  void static saveProcessingResult(
      std::string outputFilename,
      std::string inputCloudFilename,
      bool isAGoodNoseTip,
      double totalMilliseconds,
      pcl::PointXYZ originalNoseTip,
      pcl::PointXYZ foundNoseTip);

  void static saveErrorResult(
      std::string outputFilename,
      std::string inputCloudFilename,
      std::exception error);
};

#endif