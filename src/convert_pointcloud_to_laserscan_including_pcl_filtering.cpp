/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2015, Fraunhofer IPA.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 * Author: Sofie Nilsson
 */
 
/**
 This file holds a function applying a pcl statistical outlier filter to the reduced pointcloud before converting to a laserscan
 */

#include <pointcloud_to_laserscan/pointcloud_to_laserscan_including_pcl_filtering.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// includes for pcl filtering
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/range_image/range_image.h>

#include <iostream>

using namespace pointcloud_to_laserscan
{
  
  void convert_pointcloud_to_laserscan_including_pcl_filtering(const sensor_msgs::PointCloud2Ptr &cloud, sensor_msgs::LaserScan &output, double range_min, int mean_k, double std_factor )
  {
    
    // create pcl cloud objects
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    
    // convert pointcloud to pcl pointcloud
    pcl::fromROSMsg (*cloud, *pcl_cloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pcl_cloud,*pcl_cloud, indices);
    indices.clear();
    
    int n_points = pcl_cloud->size();

    // Iterate through pointcloud
    for(int i = 0; i < n_points; i++)
    {
        float* iter_x = &pcl_cloud->points[i].x;
        float* iter_y = &pcl_cloud->points[i].y;
        float* iter_z = &pcl_cloud->points[i].z;
        
        if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
        {
            NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
            continue;
        }

        //get reflection point in hight limiting planes in order to check that point lies between borders(above or below is not clearly def):
        tf2::Vector3 P(*iter_x, *iter_y, *iter_z);
        /**
         * lambda x and y describes the location within the planes, which are the same for all paralell planes with
         * aligned origin -> calculate only once for the plane at height of target frame.
         * If assumption not hold, use one set of lambda per plane (use the A value of that specific plane like this:
         *
         * double lambda_x_max =  (P - A_max_o_frame).dot(ex_o_frame);
         *
         * For now we assume tahat a common set of lambdas hold:
         */
        double lambda_x =  (P - A_target_o_frame).dot(ex_o_frame);
        double lambda_y =  (P - A_target_o_frame).dot(ey_o_frame);
        tf2::Vector3 P_max = A_max_o_frame + lambda_x*ex_o_frame + lambda_y*ey_o_frame;
        tf2::Vector3 P_min = A_min_o_frame + lambda_x*ex_o_frame + lambda_y*ey_o_frame;


        double border_distance_sqared = P_max.distance2(P_min);

        if ((P.distance2(P_max) > border_distance_sqared) || (P.distance2(P_min) > border_distance_sqared)) // Originaly: (*iter_z > max_height_ || *iter_z < min_height_)
        {
            NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
            continue;
        }

        double range = hypot(lambda_x, lambda_y);


        if (range < range_min)
        {
            NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y,
             *iter_z);
            continue;
        }

        double angle = atan2(lambda_y, lambda_x);
        if (angle < output.angle_min || angle > output.angle_max)
        {
            NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
            continue;
        }
        
        // write to reduced pointcloud
        reduced_pcl_cloud->points.push_back(pcl_cloud->points[i]);
    }
    
    // filter reduced cloud
    pcl_statistical_outlier_removal_filter(reduced_pcl_cloud, mean_k, std_factor);
    
    // project pointcloud to scan
    n_points = reduced_pcl_cloud->size();
    for (int j = 0; j < n_points; j++)
    {
        float* iter_x = &reduced_pcl_cloud->points[j].x;
        float* iter_y = &reduced_pcl_cloud->points[j].y;
        float* iter_z = &reduced_pcl_cloud->points[j].z;

        //overwrite range at laserscan ray if new range is smaller
        int index = (angle - output.angle_min) / output.angle_increment;
        if (range < output.ranges[index])
        {
            output.ranges[index] = range;
        }

    }
  }


  void pcl_statistical_outlier_removal_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, int mean_k, double std_factor)
  {
      // Create the filtering object
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud_in);
      //sor.setIndices(indices);
      sor.setMeanK (mean_k);
      sor.setStddevMulThresh (std_factor);
      sor.filter(*cloud_in);
  }
}

PLUGINLIB_DECLARE_CLASS(ipa_pointcloud_to_laserscan, IpaPointCloudToLaserScanNodelet, pointcloud_to_laserscan::IpaPointCloudToLaserScanNodelet, nodelet::Nodelet);
