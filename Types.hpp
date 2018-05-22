#ifndef TYPES
#define TYPES

#include <vector>

#include <eigen3/Eigen/Dense>


namespace ToyProblem
{
    /**
     * Aliases for making types more informative
     */
    using State = Eigen::Vector2d;
    using States = std::vector<State>;
    using Measurement = Eigen::Vector2d;
    using Measurements = std::vector<Measurement>;
    using Noise = Eigen::Vector2d;
    using Covariance = Eigen::Matrix2d;
    using Model = Eigen::Matrix2d;
    using Jacobian = Eigen::Matrix2d;
    using KalmanGain = Eigen::Matrix2d;
    using Timestep = double;
    struct Estimate
    {
        State state;
        Covariance covariance;
    };
} // namespace ToyProblem

#endif