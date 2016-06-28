
#include <pointcloud_to_laserscan/scan_outlier_removal_filter.h>

using namespace scan_outlier_filter; 

void ScanOutlierRemovalFilter::configure(double cluster_break_distance, int max_noise_cluster_size, double max_noise_cluster_distance)
{
	filter_configured_ = true;
	cluster_break_distance_ = cluster_break_distance;
	max_noise_cluster_size_ = max_noise_cluster_size;
	max_noise_cluster_distance_ = max_noise_cluster_distance;
}

void ScanOutlierRemovalFilter::remove_outliers(sensor_msgs::LaserScan &scan)
{
	if (!filter_configured_)
		return;

	// help function initialization
      int ranges_size = scan.ranges.size();
      int i_current_cluster = 0;
      bool cluster_further_away = false;
      std::vector<int> cluster_indecies;
      double diffs[ranges_size-1];

      // calculate diffs
      for (int i = 1; i< ranges_size; i++)
      {
          diffs[i-1] = scan.ranges[i] - scan.ranges[i-1];
      }

      // find and remove outliers
      for (int i = 0; i< ranges_size-1; i++)
      {
          // add point to cluster
          cluster_indecies.push_back(i);
          i_current_cluster ++;

          // check if last point in cluster; find diff larger than border -> cluster separation
          if (diffs[i] > cluster_break_distance_ || diffs[i] < -cluster_break_distance_)
          {
              bool new_cluster_further_away = (diffs[i] > 0);

              // Only remove cluster if it is closer than surrounding clusters
              if ((i_current_cluster < max_noise_cluster_size_) && (new_cluster_further_away != cluster_further_away))
              {
                  // check if all cluster points are closer than the max noise distance
                  bool is_noise_cluster = true;
                  for (int k = 0; k < i_current_cluster; k++)
                  {
                      if (scan.ranges[cluster_indecies[k]] > max_noise_cluster_distance_)
                      {
                          is_noise_cluster = false;
                          break;
                      }
                  }

                  // Only remove cluster points if all points are closer than the max noise distance
                  if(is_noise_cluster)
                  {
                      for (int k = 0; k < i_current_cluster; k++)
                      {
                          if (scan.ranges[cluster_indecies[k]] < max_noise_cluster_distance_)
                              scan.ranges[cluster_indecies[k]] = scan.range_max;
                      }
                  }
              }

              // initialize new cluster
              cluster_further_away = new_cluster_further_away;
              cluster_indecies.clear();
              i_current_cluster = 0;
          }
      }


      // remove last cluster if small enough
      if ((i_current_cluster < cluster_break_distance_) && !cluster_further_away)
      {
          for (int k = 0; k < i_current_cluster; k++)
          {
              scan.ranges[cluster_indecies[k]] = scan.range_max;
          }
          // remove last point as well
          scan.ranges[ranges_size-1] = scan.range_max;
      }
  
}