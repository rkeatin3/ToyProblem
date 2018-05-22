#include "SimpleEKF.hpp"

namespace ToyProblem{

SimpleEKF::Output SimpleEKF::operator()(Estimate state_k,
                                        Estimate process_noise_k,
                                        Measurement measurement_kp1,
                                        Estimate observation_noise_kp1,
                                        ClockStateModel state_model,
                                        ClockMeasurementModel measurement_model)
{
    // Calculate a priori state estimate
    ClockStateModel::Output state_prediction = state_model(state_k.state, process_noise_k.state);
    // Predict measurement
    ClockMeasurementModel::Output measurement_prediction = measurement_model(state_prediction.state_kp1,
                                                                             observation_noise_kp1.state);

    // Calculate innovation
    const auto Z_kp1 = measurement_kp1 - measurement_prediction.measurement_kp1;

    // Calculate a priori state covariance
    const auto& F_kp1 = state_prediction.state_jacobian;
    const auto& Q_kp1 = process_noise_k.covariance;
    const auto& P_k = state_k.covariance;
    const auto& L_kp1 = state_prediction.noise_jacobian;
    Covariance P_kp1_k = F_kp1 * P_k * F_kp1.transpose() + L_kp1 * Q_kp1 * L_kp1.transpose();

    // Calculate innovation covariance
    const auto& H_kp1 = measurement_prediction.observation_model;
    const auto& R_kp1 = observation_noise_kp1.covariance;
    const auto S_kp1  = H_kp1 * P_kp1_k * H_kp1.transpose() + R_kp1;

    // Calculate Kalman gain
    KalmanGain K_kp1 = P_kp1_k * H_kp1.transpose() * S_kp1.inverse();

    // Calculate a posteriori state estimate by adding KalmanGain * innovation to a priori prediction
    const auto X_kp1 = state_prediction.state_kp1 + K_kp1 * Z_kp1;

    // Calculate a posteriori state covariance
    const auto IminKH = Covariance::Identity() - K_kp1 * H_kp1;
    const auto P_kp1 = IminKH * P_kp1_k * IminKH.transpose() + K_kp1 * R_kp1 * K_kp1.transpose();

    // Pass filter outputs into return value
    return Output{{X_kp1, P_kp1}, {Z_kp1, S_kp1}};
}

} // namespace ToyProblem
