//
// Created by marcusv on 08/12/20.
//

#include "CloudsLog.h"
#include "Utils.h"
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

v8::Local<v8::Array> CloudsLog::toV8Array()
{
  int logsSize = logs.size();
  v8::Local<v8::Array> response = Nan::New<v8::Array>(logsSize);

  for (int i = 0; i < logsSize; i++)
  {
    CloudsLogEntry log = logs[i];

    v8::Local<v8::Object> obj = Nan::New<v8::Object>();
    obj->Set(Nan::New("cloud_label").ToLocalChecked(), Nan::New(log.cloudLabel).ToLocalChecked());
    obj->Set(Nan::New("cloud").ToLocalChecked(), Utils::parsePointCloudToV8Array(log.cloud));

    Nan::Set(response, i, obj);
  }

  return response;
}