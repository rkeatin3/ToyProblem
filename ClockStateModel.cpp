#include "ClockStateModel.hpp"

namespace ToyProblem{

ClockStateModel::Output ClockStateModel::operator()(const State& state_k,
                                                    const Noise& process_noise,
                                                    const Timestep dt)
{
    Output out;

    out.state_jacobian << 1, dt,
                          0, 1;
    out.noise_jacobian = Jacobian::Identity();
    out.state_kp1 = out.state_jacobian * state_k + out.noise_jacobian * process_noise;
    return out;
}


} // namespace ToyProblem
