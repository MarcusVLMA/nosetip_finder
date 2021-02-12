//
// Created by marcusv on 05/02/21.
//

#include <fstream>
#include "Utils.h"

void Utils::saveProcessingResult(std::string outputFilename, std::string inputCloudFilename, bool isAGoodNoseTip, double totalMilliseconds, pcl::PointXYZ originalNoseTip, pcl::PointXYZ foundNoseTip)
{
  std::ofstream outputFile;
  outputFile.open(outputFilename, std::ios_base::app);
  outputFile << inputCloudFilename << "," << isAGoodNoseTip << "," << totalMilliseconds << "," << originalNoseTip.x << "," << originalNoseTip.y << "," << originalNoseTip.z << "," << foundNoseTip.x << "," << foundNoseTip.y << "," << foundNoseTip.z << "\n";
  outputFile.close();
}

void Utils::saveErrorResult(std::string outputFilename, std::string inputCloudFilename, std::exception error)
{
  std::ofstream outputFile;
  outputFile.open(outputFilename, std::ios_base::app);
  outputFile << inputCloudFilename << "," << error.what() << "\n";
  outputFile.close();
}