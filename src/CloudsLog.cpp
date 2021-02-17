//
// Created by marcusv on 08/12/20.
//

#include "CloudsLog.h"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

std::vector<CloudsLogEntry> CloudsLog::getLogs()
{
  return logs;
}

void CloudsLog::setLogs(std::vector<CloudsLogEntry> newLogs)
{
  logs = newLogs;
}

void CloudsLog::add(std::string cloudLabel, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  struct CloudsLogEntry newLogEntry = {cloudLabel, cloud};
  logs.push_back(newLogEntry);
}