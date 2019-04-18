#include <chrono>


#include <laser_slam_ros/GetLaserTrackSrv.h>
// #include <octomap_world/octomap_manager.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>
#include <segmatch/point_color_semantics.hpp>


int main(int argc, char** argv) {
  // Initialize glog for volumetric_mapping.
  google::InitGoogleLogging(argv[0]);
  // Get output file path.
  if (argc < 2) {
    ROS_ERROR("No output file path specified");
    return -1;
  }

  // Defaults.
  double resolution = 0.075;
  double prob_hit = 0.9;
  double prob_miss = 0.4;
  double max_range = 20.0;
  bool publish = true;

  boost::posix_time::ptime my_posix_time = ros::WallTime::now().toBoost();
  std::string current_time = boost::posix_time::to_iso_extended_string(my_posix_time);
  std::string pose_log_file_name =
    "pose_offset_log_" + current_time + ".txt";
  ROS_INFO("Writing pose offsets to %s", pose_log_file_name.c_str());
  std::ofstream pose_log_file(pose_log_file_name);
  int pose_counter = 0;

  std::string file_path(argv[1]);
  file_path = file_path.substr(0, file_path.length() - 4) + current_time + ".pcd";

  // Parse arguments.
  if (argc > 2) {
    if (argc % 2) {
      ROS_ERROR("Invalid number of command line arguments");
      return -1;
    }
    for (int i = 2; i < argc; i+=2) {
      const std::string argument(argv[i]);
      if (argument == "resolution") resolution = std::atof(argv[i+1]);
      else if (argument == "probability_hit") prob_hit = std::atof(argv[i+1]);
      else if (argument == "probability_miss") prob_miss = std::atof(argv[i+1]);
      else if (argument == "sensor_max_range") max_range = std::atof(argv[i+1]);
      else {
        ROS_ERROR("Invalid command line argument \"%s\"", argv[i]);
        return -1;
      }
    }

  }
  ROS_INFO("Output file path is: \"%s\"", file_path.c_str());
  ROS_INFO("Octomap resolution is %fm", resolution);
  ROS_INFO("Probability hit/miss are: %f / %f", prob_hit, prob_miss);
  ROS_INFO("Sensor max range is %fm", max_range);

  // Init ROS things.
  ros::init(argc, argv, "laser_to_octomap");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Get laser track.
  ros::ServiceClient client;
  client = nh.serviceClient<laser_slam_ros::GetLaserTrackSrv>("/segmapper/get_laser_track");
  laser_slam_ros::GetLaserTrackSrv call;
  ROS_INFO("Requested laser track. Waiting...");
  if (!client.call(call)) {
    ROS_ERROR("Client call failed");
    return -1;
  }
  const size_t traj_length = call.response.laser_scans.size();
  ROS_INFO("Received laser track of length %lu", traj_length);
  ROS_INFO("You can shut down laser_mapper");

  // Set parameters.
  nh_private.setParam("resolution", resolution);
  nh_private.setParam("probability_hit", prob_hit);
  nh_private.setParam("probability_miss", prob_miss);
  nh_private.setParam("sensor_max_range", max_range);
  nh_private.setParam("use_tf_transforms", false);
  nh_private.setParam("robot_frame", call.response.transforms.front().header.frame_id);
  nh_private.setParam("world_frame", call.response.transforms.front().child_frame_id);

  // Volumetric Mapping expects ConstPtr.
  // Make copies so there is no double free when ConstPtr falls out of scope.
  std::vector<sensor_msgs::PointCloud2::ConstPtr> ptr_vec;
  for (auto&& scan : call.response.laser_scans) {
    ptr_vec.emplace_back(new sensor_msgs::PointCloud2(scan));
    // for (int i = 0; i <  scan.fields[i].count; i++) {
    // for (int i = 0; i <  8; i++) {
    //   std::cout << "f: " << scan.fields[i].name << " and offset: " << scan.fields[i].offset << std::endl;
    // }
    // std::cout << scan << std::endl;
    // std::cout << *ptr_vec[0] << std::endl;
    // exit(0);
  }
  std::cout << "data: ";
  for (int i = 0; i < 80; ++i) {
    std::cout << std::to_string(static_cast<uint32_t>(ptr_vec[0]->data[i])) << ", ";
  }
  std::cout << std::endl;

  // accumulate all points in one point cloud
  // pcl::PointCloud<pcl::PointXYZRGB> pclCloud, tempCloud;
  pcl::PointCloud<segmatch::PointColorSemantics> pclCloud, tempCloud;
  std::string timestamp;
  for (size_t i = 0u; i < traj_length; ++i) {
    // pcl::fromROSMsg(*ptr_vec[i], tempCloud);
    // std::cout << "First point fromROSMsg: " << tempCloud[10].semantics_rgb << std::endl;
    // for (int j = 0; j < 10; ++j) {
    //   std::cout << tempCloud[j].x << ", " << tempCloud[j].y << ", " << tempCloud[j].rgb << ", " << tempCloud[j].semantics_rgb << ", " << std::endl;
    // }
    laser_slam_ros::convert_to_pcl_point_cloud(*ptr_vec[i], &tempCloud);
    std::cout << "First point convert_to_pcl_point_cloud: " << tempCloud[0].semantics_rgb << std::endl;
    for (int j = 0; j < 10; ++j) {
      std::cout << tempCloud[j].x << ", " << tempCloud[j].y << ", "
                << std::to_string(tempCloud[j].r) << ", " << std::to_string(tempCloud[j].g) << ", " << std::to_string(tempCloud[j].b)
                // << ", " << std::to_string(tempCloud[j].semantics_r) << ", " << std::to_string(tempCloud[j].semantics_g) << ", " << std::to_string(tempCloud[j].semantics_b)
                << ", " << tempCloud[j].semantics_rgb << std::endl;
    }
    // apply tf to point cloud
    Eigen::Affine3d tf = tf2::transformToEigen(call.response.transforms[i]);
    pcl::transformPointCloud(tempCloud, tempCloud, tf);
    pclCloud += tempCloud;

    // write poses (or translations) associated with each scan to log file
    ROS_INFO("current translation:\n%f\n%f\n%f",
             call.response.transforms[i].transform.translation.x,
             call.response.transforms[i].transform.translation.y,
             call.response.transforms[i].transform.translation.z);
    // boost::posix_time::ptime timestamp = call.response.transforms[i].header.stamp.toBoost();
    timestamp = boost::posix_time::to_iso_extended_string(call.response.transforms[i].header.stamp.toBoost());
    double time_ns = call.response.transforms[i].header.stamp.sec * 1e9 + call.response.transforms[i].header.stamp.nsec;
    ROS_INFO("current translation timestamp: %s or %f and frame seq: %d\n",
              timestamp.c_str(),
              time_ns,
             call.response.transforms[i].header.seq);
    pose_log_file << i << "\n"
                  << std::fixed /* << std::setprecision(3) */ << time_ns << "\n"
                  << call.response.transforms[i].transform.translation.x << "\n"
                  << call.response.transforms[i].transform.translation.y << "\n"
                  << call.response.transforms[i].transform.translation.z << "\n\n";
    pose_counter++;
  }
  pcl::io::savePCDFileASCII (file_path, pclCloud);

  // save current config file
boost::filesystem::path config_file_path = ros::package::getPath("segmapper") + "/launch/kitti/kitti_loop_closure.yaml";
boost::filesystem::copy_file(config_file_path,
                                           "kitti_loop_closure_" + current_time + ".yaml",
                                         boost::filesystem::copy_option::overwrite_if_exists);
  ROS_INFO("Copying config file %s", config_file_path.string().c_str());
  ROS_INFO("Saved pcd file to %s.", file_path.c_str());
}
