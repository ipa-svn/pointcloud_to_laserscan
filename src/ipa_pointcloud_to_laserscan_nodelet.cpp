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

#include <pointcloud_to_laserscan/ipa_pointcloud_to_laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// includes for pcl filtering
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/range_image/range_image.h>

#include <iostream>

using namespace pointcloud_to_laserscan;

  IpaPointCloudToLaserScanNodelet::IpaPointCloudToLaserScanNodelet() {}

  void IpaPointCloudToLaserScanNodelet::onInit()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    private_nh_ = getPrivateNodeHandle();

    private_nh_.param<std::string>("target_frame", target_frame_, "");
    private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
    private_nh_.param<double>("min_height", min_height_, 0.0);
    private_nh_.param<double>("max_height", max_height_, 1.0);

    private_nh_.param<double>("angle_min", angle_min_, -M_PI / 2.0);
    private_nh_.param<double>("angle_max", angle_max_, M_PI / 2.0);
    private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 360.0);
    private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
    private_nh_.param<double>("range_min", range_min_, 0.45);
    private_nh_.param<double>("range_max", range_max_, 4.0);

    int concurrency_level;
    private_nh_.param<int>("concurrency_level", concurrency_level, 1);
    private_nh_.param<bool>("use_inf", use_inf_, true);
    
    // Settings for pcl statistical outlier filter
    private_nh_.param<bool>("with_pcl_filtering", with_pcl_filtering_, false);
    private_nh_.param<int>("mean_k", mean_k_, 10);
    private_nh_.param<double>("std_factor", std_factor_, 0.3);

    configure_filter();

    //Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
    if (concurrency_level == 1)
    {
      nh_ = getNodeHandle();
    }
    else
    {
      nh_ = getMTNodeHandle();
    }

    // Only queue one pointcloud per running thread
    if (concurrency_level > 0)
    {
      input_queue_size_ = concurrency_level;
    }
    else
    {
      input_queue_size_ = boost::thread::hardware_concurrency();
    }

    // if pointcloud target frame specified, we need to filter by transform availability
    if (!target_frame_.empty())
    {
      tf2_.reset(new tf2_ros::Buffer());
      tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
      message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
      message_filter_->registerCallback(boost::bind(&IpaPointCloudToLaserScanNodelet::cloudCb, this, _1));
      message_filter_->registerFailureCallback(boost::bind(&IpaPointCloudToLaserScanNodelet::failureCb, this, _1, _2));
    }
    else // otherwise setup direct subscription
    {
      sub_.registerCallback(boost::bind(&IpaPointCloudToLaserScanNodelet::cloudCb, this, _1));
    }

    pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10,
                                                 boost::bind(&IpaPointCloudToLaserScanNodelet::connectCb, this),
                                                 boost::bind(&IpaPointCloudToLaserScanNodelet::disconnectCb, this));
  }

  void IpaPointCloudToLaserScanNodelet::configure_filter()
  {
    NODELET_DEBUG_STREAM("configuring filter");
    // Get filter related parameters
    
    private_nh_.param<bool>("use_outlier_filter", use_outlier_filter_, false);

    double max_noise_cluster_distance;
    double cluster_break_distance;
    int max_noise_cluster_size;
    private_nh_.param<double>("max_noise_cluster_distance", max_noise_cluster_distance, 2.0);
    private_nh_.param<double>("cluster_break_distance", cluster_break_distance, 0.3);
    private_nh_.param<int>("max_noise_cluster_size", max_noise_cluster_size, 5);

    outlier_filter_.configure(cluster_break_distance, max_noise_cluster_size, max_noise_cluster_distance);
  }

  void IpaPointCloudToLaserScanNodelet::connectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
    {
      NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
      sub_.subscribe(nh_, "cloud_in", input_queue_size_);
    }
  }

  void IpaPointCloudToLaserScanNodelet::disconnectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (pub_.getNumSubscribers() == 0)
    {
      NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
      sub_.unsubscribe();
    }
  }

  void IpaPointCloudToLaserScanNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                               tf2_ros::filter_failure_reasons::FilterFailureReason reason)
  {
    NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
        << message_filter_->getTargetFramesString());

    try
    {
      geometry_msgs::TransformStamped T_geom = tf2_->lookupTransform(cloud_msg->header.frame_id, target_frame_, cloud_msg->header.stamp, ros::Duration(0.1));
      NODELET_INFO_STREAM("Transform worked at retry ");
    }
    catch (tf2::TransformException ex)
    {
      NODELET_WARN_STREAM("Transform failure again after retry in failureCB, exception: " << ex.what());
      return;
    }
  }

  void IpaPointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {
NODELET_WARN_STREAM("cb test ");
    ros::Time start_time = ros::Time::now();
    NODELET_DEBUG_STREAM("PC with timestamp from init " << cloud_msg->header.stamp.toSec() << " recevied with a delay of " << (start_time - cloud_msg->header.stamp).toSec() << " ");
    
    // convert const ptr to ptr to support downsampling
    sensor_msgs::PointCloud2Ptr cloud(boost::const_pointer_cast<sensor_msgs::PointCloud2>(cloud_msg));

    // remove leading / on frame id in case present, which is not supported by tf2
    // e.g. HACK to be compatible with non tf2 supporting bag files
    // does not do anything if the problem dies not occur -> leave for bagfile compatibility
    if(cloud_msg->header.frame_id.find_first_of("/") == 0)
    { 
      cloud->header.frame_id.erase(0,1);
    }
    // End hack for tf2 compatibility

    // Get frame tranformation
    tf2::Transform T;

    if ((!target_frame_.empty()) && !(target_frame_ == cloud_msg->header.frame_id))
    {
      try
      {
        geometry_msgs::TransformStamped T_geom = tf2_->lookupTransform(cloud_msg->header.frame_id, target_frame_, cloud_msg->header.stamp, ros::Duration(0.1));
        // Convert geometry msgs transform to tf2 transform.
        tf2::fromMsg(T_geom.transform, T);
      }
      catch (tf2::TransformException ex)
      {
        NODELET_ERROR_STREAM("Transform failure: " << ex.what());
        return;
      }
    }
    else
    {
        // target and source frame are the same
        T.setIdentity();
    }

/*
    // Transform borders and target plane to original coordinates (saved resources to not have to transform the whole point cloud)
    // A plane is described by all points fulfilling p= A + l1*e1 + l2*e2.
    // Transformation to other coordinate frame with transformation T gives: p'= T(A) + l1*T(e1) + l2*T(e2)
    // It is here assumed that both the border planes and target plane are paralell to each other -> same e1 and e2
    // Planes for max and min height: A = [0; 0; max_height]
    tf2::Vector3 A_max_t_frame(0, 0, max_height_);
    tf2::Vector3 A_max_o_frame = T(A_max_t_frame);
    NODELET_DEBUG_STREAM("A max: " << A_max_o_frame.getX() <<", "<< A_max_o_frame.getY() <<", "<< A_max_o_frame.getZ());

    tf2::Vector3 A_min_t_frame(0, 0, min_height_);
    tf2::Vector3 A_min_o_frame = T(A_min_t_frame); // want to get the orientation vector in the new coordinates
    NODELET_DEBUG_STREAM("A min: " << A_min_o_frame.getX() <<", "<< A_min_o_frame.getY() <<", "<< A_min_o_frame.getZ());

    // Transform plane basis vectors -> use only rotation
    tf2::Vector3 ex_t_frame(1, 0, 0);
    tf2::Vector3 ex_o_frame = tf2::quatRotate(T.getRotation(), ex_t_frame);
    NODELET_DEBUG_STREAM("ex min: " << ex_o_frame.getX() <<", "<< ex_o_frame.getY() <<", "<< ex_o_frame.getZ());

    tf2::Vector3 ey_t_frame(0, 1, 0);
    tf2::Vector3 ey_o_frame = tf2::quatRotate(T.getRotation(), ey_t_frame);
    NODELET_DEBUG_STREAM("ey min: " << ey_o_frame.getX() <<", "<< ey_o_frame.getY() <<", "<< ey_o_frame.getZ());

    // Plane for target scan:
    tf2::Vector3 A_target_t_frame(0, 0, 0);
    tf2::Vector3 A_target_o_frame = T(A_target_t_frame);
*/

    //build laserscan output
    sensor_msgs::LaserScan output;
    output.header = cloud_msg->header;
    if (!target_frame_.empty())
    {
      output.header.frame_id = target_frame_;
    }

    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = 0.0;
    output.scan_time = scan_time_;
    output.range_min = range_min_;
    output.range_max = range_max_;

    //determine amount of rays to create
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

    //determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if (use_inf_)
    {
      output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    }
    else
    {
      output.ranges.assign(ranges_size, output.range_max - 1.0);
    }
    
    // convert pointcloud to laserscan with or without filtering
    if(with_pcl_filtering_)
    {
      convert_pointcloud_to_laserscan_including_pcl_filtering(cloud, output, T, range_min_, mean_k_, std_factor_);
    }
    else
    {
      convert_pointcloud_to_laserscan(cloud, output, T, range_min_);
    }
    
    if(use_outlier_filter_)
    {
      outlier_filter_.remove_outliers(output);
    }

    ros::Time end_time = ros::Time::now();
    ros::Duration dur = end_time-start_time;
    NODELET_DEBUG_STREAM("Transform for PC took " << dur.toSec());

    pub_.publish(output);
  }
    
    
  void IpaPointCloudToLaserScanNodelet::convert_pointcloud_to_laserscan(const sensor_msgs::PointCloud2Ptr &cloud, sensor_msgs::LaserScan &output, 
	const tf2::Transform &T, const double range_min )
  {
    NODELET_DEBUG_STREAM("cb without pcl ");
    // Transform borders and target plane to original coordinates (saved resources to not have to transform the whole point cloud)
    // A plane is described by all points fulfilling p= A + l1*e1 + l2*e2.
    // Transformation to other coordinate frame with transformation T gives: p'= T(A) + l1*T(e1) + l2*T(e2)
    // It is here assumed that both the border planes and target plane are paralell to each other -> same e1 and e2
    // Planes for max and min height: A = [0; 0; max_height]
    tf2::Vector3 A_max_t_frame(0, 0, max_height_);
    tf2::Vector3 A_max_o_frame = T(A_max_t_frame);
    NODELET_DEBUG_STREAM("A max: " << A_max_o_frame.getX() <<", "<< A_max_o_frame.getY() <<", "<< A_max_o_frame.getZ());

    tf2::Vector3 A_min_t_frame(0, 0, min_height_);
    tf2::Vector3 A_min_o_frame = T(A_min_t_frame); // want to get the orientation vector in the new coordinates
    NODELET_DEBUG_STREAM("A min: " << A_min_o_frame.getX() <<", "<< A_min_o_frame.getY() <<", "<< A_min_o_frame.getZ());

    // Transform plane basis vectors -> use only rotation
    tf2::Vector3 ex_t_frame(1, 0, 0);
    tf2::Vector3 ex_o_frame = tf2::quatRotate(T.getRotation(), ex_t_frame);
    NODELET_DEBUG_STREAM("ex min: " << ex_o_frame.getX() <<", "<< ex_o_frame.getY() <<", "<< ex_o_frame.getZ());

    tf2::Vector3 ey_t_frame(0, 1, 0);
    tf2::Vector3 ey_o_frame = tf2::quatRotate(T.getRotation(), ey_t_frame);
    NODELET_DEBUG_STREAM("ey min: " << ey_o_frame.getX() <<", "<< ey_o_frame.getY() <<", "<< ey_o_frame.getZ());

    // Plane for target scan:
    tf2::Vector3 A_target_t_frame(0, 0, 0);
    tf2::Vector3 A_target_o_frame = T(A_target_t_frame);

    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2Iterator<float>
              iter_x(*cloud, "x"), iter_y(*cloud, "y"), iter_z(*cloud, "z");
              iter_x != iter_x.end();
              ++iter_x, ++iter_y, ++iter_z)
    {
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


        if (range < range_min_)
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

        //overwrite range at laserscan ray if new range is smaller
        int index = (angle - output.angle_min) / output.angle_increment;
        if (range < output.ranges[index])
        {
            output.ranges[index] = range;
        }

    }

  }
  
  void IpaPointCloudToLaserScanNodelet::convert_pointcloud_to_laserscan_including_pcl_filtering(const sensor_msgs::PointCloud2Ptr cloud, 
	sensor_msgs::LaserScan &output, const tf2::Transform &T, const double range_min, const int mean_k, const double std_factor )
  {
    NODELET_DEBUG_STREAM("cb with pcl k= " << mean_k << " factor= " << std_factor);
    // Transform borders and target plane to original coordinates (saved resources to not have to transform the whole point cloud)
    // A plane is described by all points fulfilling p= A + l1*e1 + l2*e2.
    // Transformation to other coordinate frame with transformation T gives: p'= T(A) + l1*T(e1) + l2*T(e2)
    // It is here assumed that both the border planes and target plane are paralell to each other -> same e1 and e2
    // Planes for max and min height: A = [0; 0; max_height]
    tf2::Vector3 A_max_t_frame(0, 0, max_height_);
    tf2::Vector3 A_max_o_frame = T(A_max_t_frame);
    NODELET_DEBUG_STREAM("A max: " << A_max_o_frame.getX() <<", "<< A_max_o_frame.getY() <<", "<< A_max_o_frame.getZ());

    tf2::Vector3 A_min_t_frame(0, 0, 0.0);//min_height_); // Uggly hack to use filter on partly reduced pointcloud still seeing the floor
    tf2::Vector3 A_min_o_frame = T(A_min_t_frame); // want to get the orientation vector in the new coordinates
    NODELET_DEBUG_STREAM("A min: " << A_min_o_frame.getX() <<", "<< A_min_o_frame.getY() <<", "<< A_min_o_frame.getZ());

    // Transform plane basis vectors -> use only rotation
    tf2::Vector3 ex_t_frame(1, 0, 0);
    tf2::Vector3 ex_o_frame = tf2::quatRotate(T.getRotation(), ex_t_frame);
    NODELET_DEBUG_STREAM("ex min: " << ex_o_frame.getX() <<", "<< ex_o_frame.getY() <<", "<< ex_o_frame.getZ());

    tf2::Vector3 ey_t_frame(0, 1, 0);
    tf2::Vector3 ey_o_frame = tf2::quatRotate(T.getRotation(), ey_t_frame);
    NODELET_DEBUG_STREAM("ey min: " << ey_o_frame.getX() <<", "<< ey_o_frame.getY() <<", "<< ey_o_frame.getZ());

    // Plane for target scan:
    tf2::Vector3 A_target_t_frame(0, 0, 0);
    tf2::Vector3 A_target_o_frame = T(A_target_t_frame);

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

    tf2::Vector3 A_min_t_frame2(0, 0, min_height_);
    tf2::Vector3 A_min_o_frame2 = T(A_min_t_frame2); // want to get the orientation vector in the new coordinates
    NODELET_DEBUG_STREAM("A min: " << A_min_o_frame2.getX() <<", "<< A_min_o_frame.getY() <<", "<< A_min_o_frame.getZ());
    
    // project pointcloud to scan
    n_points = reduced_pcl_cloud->size();
    for (int j = 0; j < n_points; j++)
    {
        float* iter_x = &reduced_pcl_cloud->points[j].x;
        float* iter_y = &reduced_pcl_cloud->points[j].y;
        float* iter_z = &reduced_pcl_cloud->points[j].z;

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
	// need to check height again do to the uggly hack above. Otherwise the previous loop would use the correct settings right away
        double lambda_x =  (P - A_target_o_frame).dot(ex_o_frame);
        double lambda_y =  (P - A_target_o_frame).dot(ey_o_frame);
	tf2::Vector3 P_max = A_max_o_frame + lambda_x*ex_o_frame + lambda_y*ey_o_frame;
        tf2::Vector3 P_min = A_min_o_frame2 + lambda_x*ex_o_frame + lambda_y*ey_o_frame;
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
        //overwrite range at laserscan ray if new range is smaller
        int index = (angle - output.angle_min) / output.angle_increment;
        if (range < output.ranges[index])
        {
            output.ranges[index] = range;
        }

    }
  }


  void IpaPointCloudToLaserScanNodelet::pcl_statistical_outlier_removal_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, const int mean_k, const double std_factor)
  {
      // Create the filtering object
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud_in);
      //sor.setIndices(indices);
      sor.setMeanK (mean_k);
      sor.setStddevMulThresh (std_factor);
      sor.filter(*cloud_in);
  }


PLUGINLIB_DECLARE_CLASS(ipa_pointcloud_to_laserscan, IpaPointCloudToLaserScanNodelet, pointcloud_to_laserscan::IpaPointCloudToLaserScanNodelet, nodelet::Nodelet);
