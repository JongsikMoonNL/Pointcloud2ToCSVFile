#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <iostream>
#include <velodyne_pointcloud/point_types.h>


pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr temp_cloud;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
  /*sensor_msgs::PointCloud2 output;
  output = *input;
  pub.publish (output);*/

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    for(int i=0; i<temp_cloud->points.size(); i++){

        //std::cout << temp_cloud->points[i].x << std::endl;

    }
}

int main (int argc, char** argv){
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  temp_cloud = pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
  ros::Subscriber sub = nh.subscribe ("velodyne_points", 1, cloud_cb);


  time_t timer = time(NULL);
  struct tm* ltime = localtime(&timer);
  char fn_date[13];
  sprintf(fn_date, "%02d%02d%02d_%02d%02d%02d", (ltime->tm_year) - 100, (ltime->tm_mon) + 1, ltime->tm_mday,
            ltime->tm_hour, ltime->tm_min, ltime->tm_sec);
  std::string fnDate(fn_date);

  std::string home_env;
  home_env = getenv("HOME");

  std::string fileName;
  fileName = home_env + "/.ros/velLog" + fnDate + ".csv";

  FILE* logFp_;
  logFp_ = fopen(fileName.c_str(), "wb");

    ros::Rate rate(20.0f);

    while (ros::ok()) {
        if (temp_cloud->points.size() != 0) {
            ros::Time t = ros::Time::now();
            fprintf(logFp_, "%ld.%ld, ", t.sec, t.nsec);
            for (int i = 0; i < temp_cloud->points.size(); i++) {
                fprintf(logFp_, "%f, %f, %f, %f, %u, ", temp_cloud->points[i].x, temp_cloud->points[i].y, temp_cloud->points[i].z, temp_cloud->points[i].intensity, temp_cloud->points[i].ring);
            }
        fprintf(logFp_, "\n");
        std::cout << temp_cloud->points[0].x << std::endl;
            std::cout << temp_cloud->points[0].y << std::endl;
            std::cout << temp_cloud->points[0].z << std::endl;
            std::cout << temp_cloud->points[0].intensity << std::endl;
            std::cout << temp_cloud->points[0].ring << std::endl;

        }
        ros::spinOnce();
        rate.sleep();
    }
}
