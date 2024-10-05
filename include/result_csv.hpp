#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>

std::string create_date() {
  time_t t = time(nullptr);
  const tm* local_time = localtime(&t);
  std::stringstream s;
  s << "20" << local_time->tm_year - 100 << "_";
  s << local_time->tm_mon + 1 << "_";
  s << local_time->tm_mday << "_";
  s << local_time->tm_hour << "_";
  s << local_time->tm_min << "_";
  s << local_time->tm_sec;
  return (s.str());
}

struct ResultCsv {
private:
  bool evaluate_error = false;
  std::ofstream csv_file;
  std::vector<double> times;
  std::vector<Eigen::Matrix4d> ground_truths;

public:
  ResultCsv(const std::string& folder_name, const std::string& gt_pose_file_name) {
    std::string file_path = folder_name + "/result.csv";
    csv_file.open(file_path);
    if (csv_file.is_open()) {
      csv_file << "file_name,exe_time,error_trans,error_rot,gt_src_x,gt_src_y,gt_src_z,gt_src_q_x,gt_src_q_y,gt_src_q_z,gt_src_q_w,gt_tar_x,gt_tar_y,gt_tar_z,gt_tar_q_x,gt_tar_q_"
                  "y,gt_tar_q_z,gt_tar_q_w,result_x,result_y,result_z,result_q_x,result_q_y,result_q_z,result_q_w,relative_x,relative_y,relative_z,relative_q_x,relative_q_y,"
                  "relative_q_z,relative_q_w\n";
    }
  }

  ~ResultCsv() {
    if (csv_file.is_open()) {
      csv_file.close();
    }
  }

  void write(
    const std::string& file_name,
    const double exe_time,
    const Eigen::Matrix4d& error,
    const Eigen::Matrix4d& gt_src,
    const Eigen::Matrix4d& gt_tar,
    const Eigen::Matrix4d& result,
    const Eigen::Matrix4d& relative) {
    if (!csv_file.is_open()) return;

    // Extract translations and quaternions
    auto extract_trans_quat = [](const Eigen::Matrix4d& mat) {
      Eigen::Quaterniond q(mat.block<3, 3>(0, 0));
      return std::make_tuple(mat(0, 3), mat(1, 3), mat(2, 3), q.x(), q.y(), q.z(), q.w());
    };

    double error_trans = error.block<3, 1>(0, 3).norm();
    double error_rot = Eigen::AngleAxisd(error.block<3, 3>(0, 0)).angle();

    auto [gt_src_x, gt_src_y, gt_src_z, gt_src_q_x, gt_src_q_y, gt_src_q_z, gt_src_q_w] = extract_trans_quat(gt_src);
    auto [gt_tar_x, gt_tar_y, gt_tar_z, gt_tar_q_x, gt_tar_q_y, gt_tar_q_z, gt_tar_q_w] = extract_trans_quat(gt_tar);
    auto [result_x, result_y, result_z, result_q_x, result_q_y, result_q_z, result_q_w] = extract_trans_quat(result);
    auto [relative_x, relative_y, relative_z, relative_q_x, relative_q_y, relative_q_z, relative_q_w] = extract_trans_quat(relative);

    csv_file << file_name << "," << exe_time << "," << error_trans << "," << error_rot << "," << gt_src_x << "," << gt_src_y << "," << gt_src_z << "," << gt_src_q_x << ","
             << gt_src_q_y << "," << gt_src_q_z << "," << gt_src_q_w << "," << gt_tar_x << "," << gt_tar_y << "," << gt_tar_z << "," << gt_tar_q_x << "," << gt_tar_q_y << ","
             << gt_tar_q_z << "," << gt_tar_q_w << "," << result_x << "," << result_y << "," << result_z << "," << result_q_x << "," << result_q_y << "," << result_q_z << ","
             << result_q_w << "," << relative_x << "," << relative_y << "," << relative_z << "," << relative_q_x << "," << relative_q_y << "," << relative_q_z << ","
             << relative_q_w << "\n";
  }
};
