#ifndef NOSETIP_FINDER_UTILS_H
#define NOSETIP_FINDER_UTILS_H

#include <nan.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Utils
{
public:
  v8::Local<v8::Array> static parsePointCloudToV8Array(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
};

#endif //NOSETIP_FINDER_UTILS_H