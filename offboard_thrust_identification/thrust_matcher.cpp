#include "thrust_matcher.hpp"
#include <uav_cpp/utils/logger.hpp>
#include <uav_cpp/parameters/yaml_parameter_parser.hpp>
#include <fstream>
#include <sstream>
#include <algorithm>

namespace uav_identification {

ThrustMatcher::ThrustMatcher(const std::string& data_folder, const std::string& initial_param_file):
    model_matcher_(sampling_time_, {"limits.max_angle", "limits.min_z_acceleration",
      "model.vehicle_mass"}),
    data_folder_(data_folder)
 {
    uav_cpp::parameters::YamlParameterParser<uav_cpp::parameters::Parameter> parser(initial_param_file);
    auto params = parser.getParameters();
    auto initial_parameters = std::make_shared<uav_cpp::parameters::ParameterMap>();
    for (const auto& [name, param] : params) {
        initial_parameters->emplace(name, param);
    }
    initial_parameters->emplace("limits.max_angle", std::make_shared<uav_cpp::parameters::Parameter>("limits.max_angle", 0.5));
    initial_parameters->emplace("limits.min_z_acceleration", std::make_shared<uav_cpp::parameters::Parameter>("limits.min_z_acceleration", 9.81));
    model_matcher_.setParameters(initial_parameters);
}

void ThrustMatcher::readOdometryData(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open odometry data file: " << file_path << std::endl;
        return;
    }

    std::string line;
    // Skip header
    std::getline(file, line);

    auto previous_timestamp = std::chrono::nanoseconds(-1);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        uav_cpp::types::OdometryStamped odometry;

        // Parse each field
        std::getline(ss, token, ',');  // timestamp
        odometry.timestamp = std::chrono::nanoseconds(std::stoll(token));
        if(odometry.timestamp <= previous_timestamp) {
            continue;
        }

        // Skipping frame_id
        std::getline(ss, token, ',');  // frame_id

        // Parse position
        std::getline(ss, token, ',');  // odometry_position_x
        odometry.position.vector.x() = std::stod(token);
        std::getline(ss, token, ',');  // odometry_position_y
        odometry.position.vector.y() = std::stod(token);
        std::getline(ss, token, ',');  // odometry_position_z
        odometry.position.vector.z() = std::stod(token);

        // Parse velocity
        std::getline(ss, token, ',');  // odometry_velocity_x
        odometry.velocity.vector.x() = std::stod(token);
        std::getline(ss, token, ',');  // odometry_velocity_y
        odometry.velocity.vector.y() = std::stod(token);
        std::getline(ss, token, ',');  // odometry_velocity_z
        odometry.velocity.vector.z() = std::stod(token);

        // Parse attitude quaternion
        std::getline(ss, token, ',');  // odometry_attitude_w
        odometry.attitude.quaternion.w() = std::stod(token);
        std::getline(ss, token, ',');  // odometry_attitude_x
        odometry.attitude.quaternion.x() = std::stod(token);
        std::getline(ss, token, ',');  // odometry_attitude_y
        odometry.attitude.quaternion.y() = std::stod(token);
        std::getline(ss, token, ',');  // odometry_attitude_z
        odometry.attitude.quaternion.z() = std::stod(token);

        // Parse angular velocity
        std::getline(ss, token, ',');  // odometry_angular_velocity_x
        odometry.angular_velocity.vector.x() = std::stod(token);
        std::getline(ss, token, ',');  // odometry_angular_velocity_y
        odometry.angular_velocity.vector.y() = std::stod(token);
        std::getline(ss, token, ',');  // odometry_angular_velocity_z
        odometry.angular_velocity.vector.z() = std::stod(token);

        odometries_.push_back(odometry);
    }

    file.close();
}

void ThrustMatcher::readThrustData(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open thrust data file: " << file_path << std::endl;
        return;
    }

    std::string line;
    // Skip header
    std::getline(file, line);

    auto previous_timestamp = std::chrono::nanoseconds(-1);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        uav_cpp::types::ThrustStamped thrust;

        // Parse each field
        std::getline(ss, token, ',');  // timestamp
        thrust.timestamp = std::chrono::nanoseconds(std::stoll(token));
        if(thrust.timestamp <= previous_timestamp) {
            continue;
        }

        // Skipping frame_id
        std::getline(ss, token, ',');  // frame_id

        // Parse thrust
        std::getline(ss, token, ',');  // thrust
        thrust.value = std::stod(token);

        thrusts_.push_back(thrust);
    }

    file.close();
}

void ThrustMatcher::computeAccelerations() {
    // Compute accelerations from odometry data
    // This is a simple finite difference approximation
    if (odometries_.size() < 2) {
        std::cerr << "Not enough odometry data to compute accelerations." << std::endl;
        return;
    }

    for (size_t i = 1; i < odometries_.size(); ++i) {
        uav_cpp::types::AccelerationStamped accel;
        double dt = (odometries_[i].timestamp - odometries_[i - 1].timestamp).count() * 1.0e-9;  // Convert ns to s

        if (dt <= 0) {
            std::cerr << "Non-positive time difference between samples." << std::endl;
            continue;
        }

        accel.timestamp = odometries_[i].timestamp;
        accel.vector = (odometries_[i].velocity.vector - odometries_[i - 1].velocity.vector) / dt;

        accelerations_.push_back(accel);
    }
}

void ThrustMatcher::resampleData() {
    if (thrusts_.empty() || odometries_.empty()) {
        std::cerr << "No data to resample." << std::endl;
        return;
    }

    // Determine the common time interval
    auto start_time = std::max(thrusts_.front().timestamp, odometries_.front().timestamp);
    auto end_time = std::min(thrusts_.back().timestamp, odometries_.back().timestamp);

    // Resample thrusts
    std::vector<uav_cpp::types::ThrustStamped> resampled_thrusts;
    bool success = uav_cpp::utils::resampleUavCppData(
        thrusts_, resampled_thrusts, sampling_time_, start_time, end_time);

    if (!success) {
        std::cerr << "Resampling thrust data failed." << std::endl;
        return;
    }

    thrusts_ = std::move(resampled_thrusts);

    // Resample odometries
    std::vector<uav_cpp::types::OdometryStamped> resampled_odometries;
    success = uav_cpp::utils::resampleUavCppData(
        odometries_, resampled_odometries, sampling_time_, start_time, end_time);

    if (!success) {
        std::cerr << "Resampling odometry data failed." << std::endl;
        return;
    }

    odometries_ = std::move(resampled_odometries);
}

void ThrustMatcher::prepareModelMatcher() {
    // Prepare inputs and outputs for the model matcher
    for (size_t i = 0; i < accelerations_.size(); ++i) {
        // Ensure we have corresponding data
        if (i >= thrusts_.size() || i >= odometries_.size()) {
            break;
        }

        // Prepare input
        uav_cpp::types::AccelerationAttitudeStamped input;
        input.timestamp = accelerations_[i].timestamp;
        input.acceleration = accelerations_[i].vector;
        input.attitude.quaternion = odometries_[i].attitude.quaternion.normalized();

        model_matcher_.addInput(input);

        // Prepare output
        uav_cpp::types::AttitudeThrustStamped output;
        output.timestamp = thrusts_[i].timestamp;
        output.thrust = thrusts_[i];
        output.attitude.quaternion = odometries_[i].attitude.quaternion.normalized();

        model_matcher_.addOutput(output);
    }

    // Add odometry data to the model matcher
    model_matcher_.addOdometries(odometries_);
}

void ThrustMatcher::performIdentification() {
    // Read data
    readOdometryData(data_folder_ + "/identification_odometry.csv");
    readThrustData(data_folder_ + "/identification_thrust_input.csv");

    // Resample data
    UAVCPP_INFO("Resampling data...");
    resampleData();

    // Compute accelerations
    UAVCPP_INFO("Computing accelerations...");
    computeAccelerations();

    // Prepare data for model matcher
    UAVCPP_INFO("Preparing model matcher...");
    prepareModelMatcher();

    // Run the model matching algorithm
    std::vector<double> optimized_parameters;
    std::vector<uav_cpp::types::AccelerationAttitudeStamped> computed_inputs;
    std::vector<uav_cpp::types::AccelerationAttitudeStamped> resampled_inputs;
    std::vector<uav_cpp::types::AttitudeThrustStamped> resampled_outputs;
    UAVCPP_INFO("Matching model...");
    model_matcher_.matchModel(optimized_parameters, computed_inputs, resampled_inputs, resampled_outputs);

    // Output the results
    std::vector<std::string> optimized_parameters_string = model_matcher_.getOptimizedParameters();
    std::cout << "Optimized parameters: ";
    for (size_t i = 0; i < optimized_parameters_string.size(); ++i) {
        std::cout << optimized_parameters_string[i] << ": ";
        std::cout << optimized_parameters[i] << std::endl;
    }
    std::cout << std::endl;


    for (const auto & input : computed_inputs) {
      UAVCPP_DATA("identification_computed_input", input);
    }
    for (const auto & input : resampled_inputs) {
      UAVCPP_DATA("identification_resampled_input", input);
    }
    for (const auto & output : resampled_outputs) {
      UAVCPP_DATA("identification_resampled_output", output);
    }
    uav_cpp::logger::LogManager::getInstance().flushAllSinks();
}

}  // namespace uav_identification

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: thrust_matcher <data_folder> <initial_param_file>" << std::endl;
        return -1;
    }

    std::string data_folder = argv[1];
    std::string initial_param_file = argv[2];

    uav_identification::ThrustMatcher matcher(data_folder, initial_param_file);

    // Perform identification
    matcher.performIdentification();

    return 0;
}