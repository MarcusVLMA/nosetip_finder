//
// Created by marcusv on 08/12/20.
//

#ifndef NOSETIP_FINDER_CLOUDSLOG_H
#define NOSETIP_FINDER_CLOUDSLOG_H

#include <string.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <nan.h>

struct CloudsLogEntry
{
  std::string cloudLabel;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

class CloudsLog
{
private:
  std::vector<CloudsLogEntry> logs;

public:
  CloudsLog(){};
  ~CloudsLog(){};

  std::vector<CloudsLogEntry> getLogs();
  void setLogs(std::vector<CloudsLogEntry> newLogs);
  void add(std::string cloudLabel, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  v8::Local<v8::Array> toV8Array();
};

#endif