#include <algorithm>
#include <cmath>
#include <iostream>

double calculate_position_rad(double target_rad, double curr_rad) {
  return target_rad - curr_rad;
}

int main() {

  double current_angle_rad = 0.2;      // where the joint is at now
  const double target_angle_rad = 1.2; // where we want the joint to be

  const double min_angle_rad = -1.0;
  const double max_angle_rad = 1.0;

  const double gain_per_second =
      2.0; // gain converts error into the requested velocity
  const double max_velocity_rad_s = 0.4;
  const double time_step_s = 0.1; // each update represents x in seconds

  if (target_angle_rad < min_angle_rad || target_angle_rad > max_angle_rad) {
    std::cerr << "Target angle is outside the permitted range. \n";
    return -1;
  }

  const double tolerance_rad = 0.01;
  const int max_steps = 40;

  for (int step = 0; step < max_steps; ++step) {

    const double position_error_rad =
        calculate_position_rad(target_angle_rad, current_angle_rad);

    if (std::abs(position_error_rad) <= tolerance_rad) {
      break;
    }

    const double elapsed_time_s = (step + 1) * time_step_s;
    const double requested_angular_velocity_rad_s =
        gain_per_second * position_error_rad;

    const double angular_velocity_rad =
        std::clamp(requested_angular_velocity_rad_s, -max_velocity_rad_s,
                   max_velocity_rad_s);

    current_angle_rad = current_angle_rad + angular_velocity_rad * time_step_s;

    std::cout << "Time: " << elapsed_time_s
              << " s | Angle: " << current_angle_rad
              << " rad | Commanded velocity: " << angular_velocity_rad
              << " rad/s\n";
  }

  const double final_error_rad =
      calculate_position_rad(target_angle_rad, current_angle_rad);

  if (std::abs(final_error_rad) <= tolerance_rad) {
    std::cout << "Target reached within tolerance. \n";
  } else {
    std::cout << "Target not reached yet. \n";
  }

  std::cout << "Current angle: " << current_angle_rad << " rad\n";
  std::cout << "Target Angle " << target_angle_rad << " rad\n";
  std::cout << "Position Error " << final_error_rad << " rad\n";

  return 0;
}
