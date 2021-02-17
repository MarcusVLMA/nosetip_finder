//
// Created by marcusv on 10/03/19.
//

#ifndef NOSETIP_FINDER_MAIN_H
#define NOSETIP_FINDER_MAIN_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include "CloudsLog.h"

struct MainResponse
{
  CloudsLog cloudsLog;
  pcl::PointXYZ noseTip;
  double executionTime;
};

class Main
{
  typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;
  typedef pcl::PointCloud<pcl::Normal> CloudNormal;
  typedef pcl::PointCloud<pcl::PrincipalCurvatures> CloudPC;

public:
  Main(){};
  ~Main(){};

  MainResponse static run(std::string filename);
};

#endif //NOSETIP_FINDER_MAIN_H
