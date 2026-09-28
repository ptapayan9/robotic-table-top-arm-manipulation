#include <iostream>
#include <memory>
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>

int main() {

  char error[1024] = {};

  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
    mj_loadXML("models/single_joint.xml", nullptr, error, sizeof(error)),
    mj_deleteModel);

  if (!model) {
    std::cerr << "could not load model: " << error << "\n";
    return 1;
  }

  if (model->nq != 1 || model->nv != 1){
    std::cerr << "This excercise requires one hinge joint.\n";
    return 1;
  }

  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
    mj_makeData(model.get()), mj_deleteData);

  if (!data) {
    std::cerr << "Could not allocate simulation state.\n";
    return 1;
  }
  
  const double target_angle_rad = 0.5;
  const double gain_per_second = 2.0;
  const double max_velocity_rad_s = 0.4;
  const double tolerance_rad = 0.01;
  const int max_steps = 2000;
  
  std::cout << "Initial joint angle: " << data->qpos[0] << " rad\n";
  for (int step=0; step < max_steps; ++step){

    const double position_error_rad = target_angle_rad - data->qpos[0];

    if (std::abs(position_error_rad) <= tolerance_rad){
      break;
    }

    const double requested_velocity_rad = gain_per_second * position_error_rad;
    data->ctrl[0] = std::clamp(requested_velocity_rad, -max_velocity_rad_s, max_velocity_rad_s);
    mj_step(model.get(), data.get());
  }

  data->ctrl[0] = 0.0; // clear velocity request
  const double final_error_rad = target_angle_rad - data->qpos[0];

  std::cout << (std::abs(final_error_rad) <= tolerance_rad
    ? "Target reached within tolerance.\n"
    : "Target not reached within the step limit.\n");

  std::cout << "Simulation time: " << data->time << " s\n";
  std::cout << "Joint angle: " << data->qpos[0] << " rad\n";
  std::cout << "Joint velocity: " << data->qvel[0] << " rad/s\n";

  return 0;
}
