#include "include/STDesc.h"
#include "include/result_csv.hpp"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <filesystem>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

struct PoseStamp {
  double time;
  Eigen::Vector3d trans;
  Eigen::Quaterniond quat;

  Eigen::Matrix4d getMatrix() const {
    Eigen::Isometry3d T_temp = Eigen::Isometry3d::Identity();
    T_temp.translation() = trans;
    T_temp.linear() = quat.toRotationMatrix();
    Eigen::Matrix4d T = T_temp.matrix();
    return T;
  }
};

std::vector<std::string> split(const std::string& input, char delimiter) {
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

std::vector<PoseStamp> readGroundTruth(const std::string& file_name) {
  std::ifstream ifs(file_name);
  std::string line;

  std::vector<PoseStamp> gt_vec;

  // skip first line
  std::getline(ifs, line);
  while (std::getline(ifs, line)) {
    std::vector<std::string> data = split(line, ',');
    std::vector<double> row;
    for (int i = 0; i < data.size(); i++) {
      row.push_back(std::stod(data[i]));
    }

    PoseStamp pose_stamp;
    double time = row[0] + row[1] / 1e9;

    pose_stamp.time = time;
    pose_stamp.trans = Eigen::Vector3d(row[2], row[3], row[4]);

    Eigen::Quaterniond quat(row[8], row[5], row[6], row[7]);
    quat.normalize();
    pose_stamp.quat = quat;

    gt_vec.emplace_back(pose_stamp);
  }

  return gt_vec;
}

std::vector<std::string> find_point_cloud_files(const std::string& path) {
  std::filesystem::path dir(path);
  std::vector<std::string> files;

  if (!std::filesystem::exists(dir)) {
    std::cout << "[ERROR] Cannot open folder" << std::endl;
    return files;
  }

  for (const auto& entry : std::filesystem::directory_iterator(dir)) {
    const std::string extension = entry.path().extension().string();
    if (extension == ".pcd") {
      files.emplace_back(entry.path().string());
    }
  }

  return files;
}

bool can_convert_to_double(const std::vector<std::string>& name_vec) {
  for (const auto& name : name_vec) {
    try {
      std::stod(std::filesystem::path(name).stem().string());
    } catch (const std::invalid_argument& e) {
      return false;
    } catch (const std::out_of_range& e) {
      return false;
    }
  }
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_point_cloud_file(const std::string& path) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  const auto extension = std::filesystem::path(path).extension().string();
  if (extension == ".pcd") {
    if (pcl::io::loadPCDFile(path, *cloud_ptr) == -1) {
      std::cout << "[WARN] Can not open pcd file: " << path << std::endl;
      return cloud_ptr;
    }
  }
  return cloud_ptr;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_kitti");
  ros::NodeHandle nh;
  std::string pcd_folder_path;
  std::string pose_path;
  std::string output_folder_path;
  nh.param<std::string>("pcd_folder_path", pcd_folder_path, "");
  nh.param<std::string>("pose_path", pose_path, "");
  nh.param<std::string>("output_folder_path", output_folder_path, "");

  ConfigSetting config_setting;
  read_parameters(nh, config_setting);

  ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  ros::Publisher pubCureentCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  ros::Publisher pubCurrentCorner = nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  ros::Publisher pubMatchedCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  ros::Publisher pubMatchedCorner = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  ros::Publisher pubSTD = nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);

  STDescManager* std_manager = new STDescManager(config_setting);

  // read ground truth
  const auto gt_vec = readGroundTruth(pose_path);

  // Create output folder with date and create result csv
  std::string date = create_date();
  std::string pcd_save_folder_path = output_folder_path + "/" + date;
  std::filesystem::create_directory(pcd_save_folder_path);
  ResultCsv result_csv(pcd_save_folder_path, pose_path);

  // read pcd files
  auto src_cloud_files = find_point_cloud_files(pcd_folder_path);

  // sort src files
  if (can_convert_to_double(src_cloud_files)) {
    std::sort(src_cloud_files.begin(), src_cloud_files.end(), [](const std::string& a, const std::string& b) {
      return std::stod(std::filesystem::path(a).stem().string()) < std::stod(std::filesystem::path(b).stem().string());
    });
  }

  std::vector<std::vector<STDesc>> stds_vec_stock;
  std::vector<std::string> file_name_vec;
  std::vector<Eigen::Matrix4d> gt_pose_stamps_vec;
  stds_vec_stock.reserve(src_cloud_files.size());
  file_name_vec.reserve(src_cloud_files.size());
  gt_pose_stamps_vec.reserve(src_cloud_files.size());

  // file loop
  for (const auto& src_cloud_file : src_cloud_files) {
    // get time from filename
    std::string file_name = std::filesystem::path(src_cloud_file).filename().string();
    double time_stamp = std::stod(file_name);

    // find gt pose stamp
    auto min_error_iter = std::min_element(gt_vec.begin(), gt_vec.end(), [time_stamp](const PoseStamp& a, const PoseStamp& b) {
      return std::abs(a.time - time_stamp) < std::abs(b.time - time_stamp);
    });
    int index = std::distance(gt_vec.begin(), min_error_iter);
    auto gt_pose_stamp = gt_vec[index];

    if (std::abs(time_stamp - gt_pose_stamp.time) > 0.01) {
      std::cout << std::fixed << std::setprecision(18) << "[Missing]time_stamp: " << time_stamp << ", gt_pose_stamp.time: " << gt_pose_stamp.time << std::endl;
      continue;
    }

    // load and transform point cloud
    auto cloud = load_point_cloud_file(src_cloud_file);
    const Eigen::Matrix4d gt_T = gt_pose_stamp.getMatrix();
    pcl::transformPointCloud(*cloud, *cloud, gt_T.cast<float>());

    // insert
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*cloud, *temp_cloud);
    down_sampling_voxel(*temp_cloud, config_setting.ds_size_);

    // Descriptor Extraction
    std::vector<STDesc> stds_vec;
    std_manager->GenerateSTDescs(temp_cloud, stds_vec);
    stds_vec_stock.emplace_back(stds_vec);
    file_name_vec.emplace_back(file_name);
    gt_pose_stamps_vec.emplace_back(gt_T);

    // Add descriptors to the database
    std_manager->AddSTDescs(stds_vec);

    // save key
    pcl::PointCloud<pcl::PointXYZI> save_key_cloud;
    save_key_cloud = *temp_cloud;
    std_manager->key_cloud_vec_.push_back(save_key_cloud.makeShared());

    // publish
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(*temp_cloud, pub_cloud);
    pub_cloud.header.frame_id = "camera_init";
    pubCureentCloud.publish(pub_cloud);

    pcl::toROSMsg(*std_manager->corner_cloud_vec_.back(), pub_cloud);
    pub_cloud.header.frame_id = "camera_init";
    pubCurrentCorner.publish(pub_cloud);

    nav_msgs::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.pose.pose.position.x = gt_pose_stamp.trans.x();
    odom.pose.pose.position.y = gt_pose_stamp.trans.y();
    odom.pose.pose.position.z = gt_pose_stamp.trans.z();
    odom.pose.pose.orientation.w = gt_pose_stamp.quat.w();
    odom.pose.pose.orientation.x = gt_pose_stamp.quat.x();
    odom.pose.pose.orientation.y = gt_pose_stamp.quat.y();
    odom.pose.pose.orientation.z = gt_pose_stamp.quat.z();
    pubOdomAftMapped.publish(odom);
  }

  // Searching Loop
  for (int i = 0; i < stds_vec_stock.size(); i++) {
    const auto start_time = std::chrono::system_clock::now();

    std::pair<int, double> search_result(-1, 0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
    loop_transform.first << 0, 0, 0;
    loop_transform.second = Eigen::Matrix3d::Identity();

    std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
    std_manager->SearchLoop(stds_vec_stock[i], search_result, loop_transform, loop_std_pair);

    const auto end_time = std::chrono::system_clock::now();
    const double elapsed_time_msec = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1e6;

    if (search_result.first < 0) continue;

    std::cout << "key id " << search_result.first << "query id " << i << std::endl;
    const std::string matched_name = file_name_vec[i];
    const Eigen::Matrix4d gt_T_world_target = gt_pose_stamps_vec[search_result.first];
    const Eigen::Matrix4d gt_T_world_source = gt_pose_stamps_vec[i];
    Eigen::Isometry3d relative_T = Eigen::Isometry3d::Identity();
    relative_T.translation() = loop_transform.first;
    relative_T.linear() = loop_transform.second;

    result_csv.write(std::filesystem::path(matched_name).stem().string(), elapsed_time_msec, gt_T_world_source, gt_T_world_target, relative_T.matrix());

    std::string matched_folder_path = pcd_save_folder_path + "/" + matched_name;
    if (!std::filesystem::exists(matched_folder_path)) {
      std::filesystem::create_directory(matched_folder_path);
    }

    pcl::PointCloud<pcl::PointXYZI> target_cloud = *std_manager->key_cloud_vec_[search_result.first];
    pcl::io::savePCDFileBinary(matched_folder_path + "/target.pcd", target_cloud);

    pcl::PointCloud<pcl::PointXYZI> source_cloud = *std_manager->key_cloud_vec_[i];
    pcl::io::savePCDFileBinary(matched_folder_path + "/source.pcd", source_cloud);

    // publish
    if (search_result.first > 0) {
      sensor_msgs::PointCloud2 pub_cloud;
      pcl::toROSMsg(*std_manager->key_cloud_vec_[search_result.first], pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubMatchedCloud.publish(pub_cloud);
      pcl::toROSMsg(*std_manager->corner_cloud_vec_[search_result.first], pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubMatchedCorner.publish(pub_cloud);
      publish_std_pairs(loop_std_pair, pubSTD);

      pcl::toROSMsg(source_cloud, pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCureentCloud.publish(pub_cloud);
      pcl::toROSMsg(*std_manager->corner_cloud_vec_[i], pub_cloud);
      pub_cloud.header.frame_id = "camera_init";
      pubCurrentCorner.publish(pub_cloud);
    } else {
      std::cout << "[Search] no loop" << std::endl;
    }
  }
  return 0;
}