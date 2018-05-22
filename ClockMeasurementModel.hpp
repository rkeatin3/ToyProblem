#ifndef CLOCK_MEAUREMENTS
#define CLOCK_MEAUREMENTS

#include "Types.hpp"

namespace ToyProblem{

/**
 * Simple functor for estimating measurement at each timestep. This class contains no state (like the filter
 * provided in the Matlab code)).
 */
class ClockMeasurementModel
{
 public:
    struct Output
    {
        Measurement measurement_kp1; // Measurement estimate
        Model observation_model; // Observation model
    };

    Output operator()(const State& state_kp1, // A posteriori state estimate
                      const Noise& measurement_noise_kp1); // Measurement noise
};

} // namespace ToyProblem

#endif