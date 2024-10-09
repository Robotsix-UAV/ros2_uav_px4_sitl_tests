#pragma once

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <iostream>
#include <Eigen/Dense>
#include <uav_cpp/parameters/parameter.hpp>
#include <uav_cpp/model_identification/model_matcher.hpp>
#include <uav_cpp/vehicle_models/quadrotor.hpp>
#include <uav_cpp/vehicle_models/thrust_scaler.hpp>
#include <uav_cpp/vehicle_models/lift_drag.hpp>
#include <uav_cpp/types/timestamped_types.hpp>

namespace uav_identification {

using uav_cpp::identification::ModelMatcher;
using uav_cpp::models::QuadrotorModel;
using uav_cpp::models::LiftDragQuaternion;
using AttitudeThrustScaler =
    uav_cpp::models::ThrustScalerWrapper<uav_cpp::types::AttitudeThrustStamped>;

class ThrustMatcher{
public:
    ThrustMatcher(const std::string& data_folder, const std::string& initial_param_file);

    void performIdentification();

private:
    void readOdometryData(const std::string& file_path);
    void readThrustData(const std::string& file_path);
    void resampleData();
    void prepareModelMatcher();
    void computeAccelerations();

    std::vector<uav_cpp::types::ThrustStamped> thrusts_;
    std::vector<uav_cpp::types::OdometryStamped> odometries_;
    std::vector<uav_cpp::types::AccelerationStamped> accelerations_;
    const std::chrono::milliseconds sampling_time_ = std::chrono::milliseconds(100);
    uav_cpp::identification::ModelMatcher<QuadrotorModel,
        AttitudeThrustScaler> model_matcher_;
    std::string data_folder_;
};

}  // namespace uav_identification
