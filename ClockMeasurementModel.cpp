#include "ClockMeasurementModel.hpp"

namespace ToyProblem{

ClockMeasurementModel::Output ClockMeasurementModel::operator()(const State& state_kp1,
                                                                const Noise& measurement_noise_kp1)
{
    Output out;
    out.observation_model = Model::Identity();
    out.measurement_kp1 = out.observation_model * state_kp1 + measurement_noise_kp1;
    return out;
}

} // namespace ToyProblem