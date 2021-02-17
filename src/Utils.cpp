//
// Created by marcusv on 05/02/21.
//

#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include "Utils.h"

CloudXYZ::Ptr Utils::loadCloudFile(std::string filename)
{
  CloudXYZ::Ptr cloud(new CloudXYZ);

  if (filename.substr(filename.length() - 3) == "pcd")
  {
    if (pcl::io::loadPCDFile(filename, *cloud) == -1)
    {
      throw std::runtime_error("Couldn't read PCD file");
    }
  }
  else if (filename.substr(filename.length() - 3) == "obj")
  {
    if (pcl::io::loadOBJFile(filename, *cloud) == -1)
    {
      throw std::runtime_error("Couldn't read OBJ file");
    }
  }
  else if (filename.substr(filename.length() - 3) == "ply")
  {
    if (pcl::io::loadPLYFile(filename, *cloud) == -1)
    {
      throw std::runtime_error("Couldn't read PLY file");
    }
  }

  return cloud;
}

void Utils::saveProcessingResult(std::string outputFilename, std::string inputCloudFilename, bool isAGoodNoseTip, double totalMilliseconds, pcl::PointXYZ originalNoseTip, pcl::PointXYZ foundNoseTip)
{
  std::ofstream outputFile;
  outputFile.open(outputFilename, std::ios_base::app);
  outputFile << inputCloudFilename << "," << isAGoodNoseTip << "," << totalMilliseconds << "," << originalNoseTip.x << "," << originalNoseTip.y << "," << originalNoseTip.z << "," << foundNoseTip.x << "," << foundNoseTip.y << "," << foundNoseTip.z << "\n";
  outputFile.close();
}

void Utils::saveProcessingResult(std::string outputFilename, std::string inputCloudFilename, bool isAGoodNoseTip, double totalMilliseconds, pcl::PointXYZ foundNoseTip)
{
  std::ofstream outputFile;
  outputFile.open(outputFilename, std::ios_base::app);
  outputFile << inputCloudFilename << "," << isAGoodNoseTip << "," << totalMilliseconds << "," << foundNoseTip.x << "," << foundNoseTip.y << "," << foundNoseTip.z << "\n";
  outputFile.close();
}

void Utils::saveErrorResult(std::string outputFilename, std::string inputCloudFilename, std::string error)
{
  std::ofstream outputFile;
  outputFile.open(outputFilename, std::ios_base::app);
  outputFile << inputCloudFilename << "," << error << "\n";
  outputFile.close();
}