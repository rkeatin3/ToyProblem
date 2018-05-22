#ifndef CLOCK_STATE_MODEL
#define CLOCK_STATE_MODEL

#include "Types.hpp"

namespace ToyProblem{

/**
 * Simple functor for estimating state at each timestep. This class contains no state (like the filter
 * provided in the Matlab code)).
 */
class ClockStateModel
{
 public:

    // Outout for each step of prediction
    struct Output
    {
        State state_kp1; // Updated state estimate
        Jacobian state_jacobian; // Jacobian relating state estimate to previous state
        Jacobian noise_jacobian; // Jacobian relating state estimate to previous noise
    };

    Output operator()(const State& state_k, // A priori state estimate
                      const Noise& process_noise, // A priori noise estimate
                      const Timestep = 1); // dt
};

} // namespace ToyProblem

#endif