#ifndef LASER_SLAM_ROS_COMMON_HPP_
#define LASER_SLAM_ROS_COMMON_HPP_

#include <laser_slam/benchmarker.hpp>
#include <laser_slam/parameters.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

#include <segmatch/point_color_semantics.hpp>

namespace laser_slam_ros {

// typedef pcl::PointXYZRGB PclPoint;
typedef segmatch::PointColorSemantics PclPoint;
typedef pcl::PointCloud<PclPoint> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointICloud;
typedef PointICloud::Ptr PointICloudPtr;

struct LaserSlamWorkerParams {
  // Map creation & filtering parameters.
  double distance_to_consider_fixed;
  bool separate_distant_map;
  bool create_filtered_map;
  double minimum_distance_to_add_pose;
  double voxel_size_m;
  int minimum_point_number_per_voxel;


  bool remove_ground_from_local_map = false;
  double ground_distance_to_robot_center_m;

  bool use_odometry_information = true;


  // Frames.
  std::string odom_frame;
  std::string sensor_frame;
  std::string world_frame;

  // Topics.
  std::string assembled_cloud_sub_topic;
  std::string trajectory_pub_topic;
  std::string odometry_trajectory_pub_topic;
  std::string full_map_pub_topic;
  std::string local_map_pub_topic;
  std::string distant_map_pub_topic;
  std::string get_laser_track_srv_topic;

  // Map publication.
  bool publish_local_map;
  bool publish_full_map;
  bool publish_distant_map;
  double map_publication_rate_hz;
}; // struct LaserSlamWorkerParams

static LaserSlamWorkerParams getLaserSlamWorkerParams(const ros::NodeHandle& nh,
                                                      const std::string& prefix) {
  LaserSlamWorkerParams params;
  const std::string ns = prefix + "/LaserSlamWorker";

  nh.getParam(ns + "/distance_to_consider_fixed", params.distance_to_consider_fixed);
  nh.getParam(ns + "/separate_distant_map", params.separate_distant_map);
  nh.getParam(ns + "/create_filtered_map", params.create_filtered_map);
  nh.getParam(ns + "/minimum_distance_to_add_pose", params.minimum_distance_to_add_pose);
  nh.getParam(ns + "/voxel_size_m", params.voxel_size_m);
  nh.getParam(ns + "/minimum_point_number_per_voxel", params.minimum_point_number_per_voxel);


  nh.getParam(ns + "/remove_ground_from_local_map", params.remove_ground_from_local_map);
  nh.getParam(ns + "/ground_distance_to_robot_center_m", params.ground_distance_to_robot_center_m);

  nh.getParam(ns + "/use_odometry_information", params.use_odometry_information);

  nh.getParam(ns + "/odom_frame", params.odom_frame);
  nh.getParam(ns + "/sensor_frame", params.sensor_frame);
  nh.getParam(ns + "/world_frame", params.world_frame);

  nh.getParam(ns + "/publish_local_map", params.publish_local_map);
  nh.getParam(ns + "/publish_full_map", params.publish_full_map);
  nh.getParam(ns + "/publish_distant_map", params.publish_distant_map);
  nh.getParam(ns + "/map_publication_rate_hz", params.map_publication_rate_hz);

  nh.getParam(ns + "/assembled_cloud_sub_topic", params.assembled_cloud_sub_topic);
  nh.getParam(ns + "/trajectory_pub_topic", params.trajectory_pub_topic);
  nh.getParam(ns + "/odometry_trajectory_pub_topic", params.odometry_trajectory_pub_topic);
  nh.getParam(ns + "/full_map_pub_topic", params.full_map_pub_topic);
  nh.getParam(ns + "/local_map_pub_topic", params.local_map_pub_topic);
  nh.getParam(ns + "/distant_map_pub_topic", params.distant_map_pub_topic);
  nh.getParam(ns + "/get_laser_track_srv_topic", params.get_laser_track_srv_topic);

  return params;
}

static laser_slam::LaserTrackParams getLaserTrackParams(const ros::NodeHandle& nh,
                                                        const std::string& prefix) {
  laser_slam::LaserTrackParams params;
  const std::string ns = prefix + "/LaserTrack";

  std::vector<float> odometry_noise_model, icp_noise_model;
  constexpr unsigned int kNoiseModelDimension = 6u;
  nh.getParam(ns + "/odometry_noise_model", odometry_noise_model);
  CHECK_EQ(odometry_noise_model.size(), kNoiseModelDimension);
  for (size_t i = 0u; i < 6u; ++i) {
    params.odometry_noise_model[i] = odometry_noise_model.at(i);
  }
  nh.getParam(ns + "/icp_noise_model", icp_noise_model);
  CHECK_EQ(icp_noise_model.size(), kNoiseModelDimension);
  for (size_t i = 0u; i < 6u; ++i) {
    params.icp_noise_model[i] = icp_noise_model.at(i);
  }
  nh.getParam(ns + "/add_m_estimator_on_odom", params.add_m_estimator_on_odom);
  nh.getParam(ns + "/add_m_estimator_on_icp", params.add_m_estimator_on_icp);

  // TODO move loading of icp_configuration_file and icp_input_filters_file to here.
  nh.getParam(ns + "/use_icp_factors", params.use_icp_factors);
  nh.getParam(ns + "/use_odom_factors", params.use_odom_factors);
  nh.getParam(ns + "/nscan_in_sub_map", params.nscan_in_sub_map);
  nh.getParam(ns + "/save_icp_results", params.save_icp_results);

  nh.getParam(ns + "/force_priors", params.force_priors);
  return params;
}

static laser_slam::EstimatorParams getOnlineEstimatorParams(const ros::NodeHandle& nh,
                                                            const std::string& prefix) {
  laser_slam::EstimatorParams params;
  const std::string ns = prefix + "/OnlineEstimator";

  std::vector<float>  loop_closure_noise_model;
  constexpr unsigned int kNoiseModelDimension = 6u;
  nh.getParam(ns + "/loop_closure_noise_model", loop_closure_noise_model);
  CHECK_EQ(loop_closure_noise_model.size(), kNoiseModelDimension);
  for (size_t i = 0u; i < 6u; ++i) {
    params.loop_closure_noise_model[i] = loop_closure_noise_model.at(i);
  }
  nh.getParam(ns + "/add_m_estimator_on_loop_closures", params.add_m_estimator_on_loop_closures);

  nh.getParam(ns + "/do_icp_step_on_loop_closures", params.do_icp_step_on_loop_closures);
  nh.getParam(ns + "/loop_closures_sub_maps_radius", params.loop_closures_sub_maps_radius);

  params.laser_track_params = getLaserTrackParams(nh, ns);

  return params;
}

static laser_slam::BenchmarkerParams getBenchmarkerParams(const ros::NodeHandle& nh,
                                                          const std::string& prefix) {
  laser_slam::BenchmarkerParams params;
  const std::string ns = prefix + "/Benchmarker";

  nh.getParam(ns + "/save_statistics_only", params.save_statistics_only);
  nh.getParam(ns + "/enable_live_output", params.enable_live_output);
  nh.getParam(ns + "/results_directory", params.results_directory);

  return params;
}

static PointCloud lpmToPcl(const laser_slam::PointMatcher::DataPoints& cloud_in) {
  PointCloud cloud_out;
  cloud_out.width = cloud_in.getNbPoints();
  cloud_out.height = 1;
  for (size_t i = 0u; i < cloud_in.getNbPoints(); ++i) {
    PclPoint point;
    point.x = cloud_in.features(0,i);
    point.y = cloud_in.features(1,i);
    point.z = cloud_in.features(2,i);
    // TODO(ben): add RGB info here
    if (cloud_in.descriptorExists("color")) {
      assert(cloud_in.descriptors(0,i) * 255 < 256);
      uint8_t color_starting_row = cloud_in.getDescriptorStartingRow("color");
      uint8_t r = cloud_in.descriptors(color_starting_row, i) * 255;
      uint8_t g = cloud_in.descriptors(color_starting_row + 1, i) * 255;
      uint8_t b = cloud_in.descriptors(color_starting_row + 2, i) * 255;

      uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
      point.rgb = *reinterpret_cast<float*>(&rgb);
      // std::cout << "r: " << std::to_string(r) << " g: " << std::to_string(g) << " b: " << std::to_string(b) << std::endl;
    } else {
        std::cout << "No color descriptors present in laser_slam!" << std::endl;
    }

    if (cloud_in.descriptorExists("semantics_rgb")) {
      assert(cloud_in.descriptors(0,i) * 255 < 256);
      uint32_t semantics_rgb = cloud_in.descriptors(cloud_in.getDescriptorStartingRow("semantics_rgb"), i);

      // std::cout << "dim: " << cloud_in.getDescriptorDimension("semantics_rgb") << " and semantics_rgb: " << std::to_string(semantics_rgb) << std::endl;
      point.semantics_rgb = semantics_rgb;

    } else {
        std::cout << "No semantics descriptors present in laser_slam!" << std::endl;
    }
    cloud_out.push_back(point);
  }
  return cloud_out;
}

template<typename PointCloudT>
static void convert_to_pcl_point_cloud(const sensor_msgs::PointCloud2& cloud_message,
                                       PointCloudT* converted) {

  // pcl::PCDWriter w;
  // w.write ("tempddd.pcd", cloud_message, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  // w.writeASCII("tempddd.pcd", cloud_message);
  pcl::io::savePCDFile ("intermediate.pcd", cloud_message, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  pcl::PCLPointCloud2 pcl_point_cloud_2;
  std::cout << "1\n";
  pcl_conversions::toPCL(cloud_message.header, pcl_point_cloud_2.header);
  std::cout << "2\n";
  pcl_conversions::toPCL(cloud_message.fields, pcl_point_cloud_2.fields);
  std::cout << "3\n";
  pcl_conversions::toPCL(cloud_message, pcl_point_cloud_2);
  std::cout << "4\n";
  pcl::io::savePCDFile("pc2.pcd", pcl_point_cloud_2);

  // std::cout << "fields in conversion:\n";
  std::cout << "pcl_point_cloud_2: " << /* pcl_point_cloud_2 << */ std::endl;

  std::cout << "fields[]" << std::endl;
  for (size_t i = 0; i < pcl_point_cloud_2.fields.size (); ++i)
  {
    // if (pcl_point_cloud_2.fields[i].name == "semantics_rgb") {
    //   pcl_point_cloud_2.fields[i].datatype = 6;
    // }
    std::cout << "  fields[" << i << "]: ";
    std::cout << std::endl;
    std::cout << "    " << pcl_point_cloud_2.fields[i].name << " --> " << std::to_string(pcl_point_cloud_2.fields[i].datatype) << std::endl;
  }
  std::cout << "data[]" << std::endl;
  for (size_t i = 0; i < 100 /* pcl_point_cloud_2.data.size() */; ++i)
  {
    std::cout << "  data[" << i << "]: ";
    std::cout << "  " << std::to_string(pcl_point_cloud_2.data[i]) << std::endl;
  }
  // for (auto& f : pcl_point_cloud_2.fields) {
  //     std::cout << "f2: " <<  f.name << std::endl;
  // }
  // std::cout << "msg: " << std::endl;
  // // for (int i = 0; i < converted->row_step * converted->height; ++i) {
  // //   if (converted->row_step * converted->height > 22) {
  // for (int i = 0; i < 80; ++i) {
  //       std::cout << std::to_string(pcl_point_cloud_2.data[i]) << " ";
  // }
    // }
  // std::cout << pcl_point_cloud_2;
  // std::cout << std::endl;
  pcl::fromPCLPointCloud2(pcl_point_cloud_2, *converted);
  uint32_t point_semantics_rgb = *reinterpret_cast<int*>(&(*converted)[0].semantics_rgb);
  uint32_t centroid_r = (point_semantics_rgb >> 16) & 0xff;
  uint32_t centroid_g = (point_semantics_rgb >> 8) & 0xff;
  uint32_t centroid_b = point_semantics_rgb & 0xff;
  std::cout << "First point converted: " << std::to_string(centroid_r) << ", "
            << std::to_string(centroid_g) << ", " << std::to_string(centroid_b) << ", " << std::endl;
  std::cout << *converted << std::endl;
}

template<typename PointCloudT>
static void convert_to_point_cloud_2_msg(const PointCloudT& cloud,
                                         const std::string& frame,
                                         sensor_msgs::PointCloud2* converted) {
  CHECK_NOTNULL(converted);
  // Convert to PCLPointCloud2.
  pcl::PCLPointCloud2 pcl_point_cloud_2;
  pcl::toPCLPointCloud2(cloud, pcl_point_cloud_2);

  // std::string red = "\033[0;31m";
  // std::string reset = "\033[0m";
//   std::cout << "Showing cloud contents (len: " << cloud.size() << "): " << std::endl;
//   for (int i = 0; i < cloud.size(); ++i) {
//     // std::cout << std::to_string(cloud.points[i].rgb) << " ";
//     std::cout << std::to_string(cloud.points[i].r) << ", " << std::to_string(cloud.points[i].g) << ", " << std::to_string(cloud.points[i].b) << " || ";
//   }
//   std::cout << std::endl;

//   std::cout << "Showing pcl_point_cloud_2 contents " << std::endl;
//   for (int i = 0; i < pcl_point_cloud_2.row_step * pcl_point_cloud_2.height; ++i) {
//     std::cout << std::to_string(pcl_point_cloud_2.data[i]) << " ";
//   }
//   std::cout << std::endl;

//   if (cloud.size() > 0) {
//       uint8_t cloud_data[sizeof(cloud.points[0])];
//       std::cout << "sizeof(cloud.points[0]): " << std::to_string(sizeof(cloud.points[0]));
//       memcpy(&cloud_data[0], &cloud.points[0], sizeof(cloud.points[0]));

//       for (int i = 0; i < sizeof(cloud.points[0]); ++i) {
//         std::cout << std::to_string(cloud_data[i]) << " ";
//       }
//   }
//   std::cout << std::endl;


  // Convert to sensor_msgs::PointCloud2.
  pcl_conversions::fromPCL(pcl_point_cloud_2, *converted);
  // Apply frame to msg.
  converted->header.frame_id = frame;

//   std::cout << "msg fields: " << std::endl;
//   for (int i = 0; i < 7/* converted->fields.count */; ++i) {
//         std::cout << converted->fields[i].name << " ";
//   }
//   std::cout << std::endl;

}

static void applyCylindricalFilter(const PclPoint& center, double radius_m,
                                   double height_m, bool remove_point_inside,
                                   PointCloud* cloud) {
  CHECK_NOTNULL(cloud);
  PointCloud filtered_cloud;

  const double radius_squared = pow(radius_m, 2.0);
  const double height_halved_m = height_m / 2.0;

  for (size_t i = 0u; i < cloud->size(); ++i) {
    if (remove_point_inside) {
      if ((pow(cloud->points[i].x - center.x, 2.0)
          + pow(cloud->points[i].y - center.y, 2.0)) >= radius_squared ||
          abs(cloud->points[i].z - center.z) >= height_halved_m) {
        filtered_cloud.points.push_back(cloud->points[i]);
      }
    } else {
      if ((pow(cloud->points[i].x - center.x, 2.0)
          + pow(cloud->points[i].y - center.y, 2.0)) <= radius_squared &&
          abs(cloud->points[i].z - center.z) <= height_halved_m) {
        filtered_cloud.points.push_back(cloud->points[i]);
      }
    }
  }

  filtered_cloud.width = 1;
  filtered_cloud.height = filtered_cloud.points.size();

  *cloud = filtered_cloud;
}

} // namespace laser_slam_ros

#endif // LASER_SLAM_ROS_COMMON_HPP_
