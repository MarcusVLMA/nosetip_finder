#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "Utils.h"

v8::Local<v8::Array> Utils::parsePointCloudToV8Array(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{
  int pointCloudSize = pointCloud->points.size();
  v8::Local<v8::Array> response = Nan::New<v8::Array>(pointCloudSize);

  for (int i = 0; i < pointCloudSize; i++)
  {
    pcl::PointXYZ point;
    v8::Local<v8::Object> obj = Nan::New<v8::Object>();

    point = pointCloud->points[i];

    obj->Set(Nan::New("x").ToLocalChecked(), Nan::New<v8::Number>(point.x));
    obj->Set(Nan::New("y").ToLocalChecked(), Nan::New<v8::Number>(point.y));
    obj->Set(Nan::New("z").ToLocalChecked(), Nan::New<v8::Number>(point.z));
    Nan::Set(response, i, obj);
  }

  return response;
}
