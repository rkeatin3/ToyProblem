#ifndef SIMPLE_EKF
#define SIMPLE_EKF

#include "Types.hpp"
#include "ClockStateModel.hpp"
#include "ClockMeasurementModel.hpp"

namespace ToyProblem{

/**
 * Simple functor for running filter at each timestep. This class contains no state (like the filter
 * provided in the Matlab code)).
 */
class SimpleEKF
{
 public:

    // Outout for each step of filtering process
    struct Output
    {
        Estimate state_kp1; // A posteriori state estimate
        Estimate innovation_kp1; // Innovation of latest measurement (difference between expected and actual measurement)
    };

    Output operator()(Estimate state_k, // A posterior estimate from last timestep
                      Estimate process_noise_k, // A priori process noise estimate
                      Measurement measurement_kp1, // Newest measurement
                      Estimate observation_noise_kp1, // Observation noise estimate
                      ClockStateModel state_model, // State dynamics model
                      ClockMeasurementModel measurement_model); // Measurement model
};

} // namespace ToyProblem

#endif