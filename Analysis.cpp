#include "Parser.hpp"
#include "SimpleEKF.hpp"
#include "ClockStateModel.hpp"
#include "ClockMeasurementModel.hpp"

/**
 * Executable to carry out C++ equivalent calculations to process_filter_input.m.
 * Usage: analysis <DirectoryHoldingDataFiles>
 */
int main(int argc, char* argv[])
{
    using namespace ToyProblem;
    std::string data_dir(argv[1]);

    // Load noise and measurement covariances
    CovarianceParser process_noise_cov_parser(data_dir + "/process_noise_covariance.txt");
    Covariance clock_noise_cov = process_noise_cov_parser.parse();
    CovarianceParser measurement_noise_cov_parser(data_dir + "/measurement_noise_covariance.txt");
    Covariance measurement_noise_cov = measurement_noise_cov_parser.parse();

    // Load initial state estimate
    StateParser initial_state_parser(data_dir + "/initial_state_estimate.txt");
    State xk_est = initial_state_parser.parse();
    CovarianceParser initial_cov_parser(data_dir + "/initial_state_estimate_covariance.txt");
    Covariance xcovk_est = initial_cov_parser.parse();

    // Load measurements
    MeasurementParser measurement_parser(data_dir + "/measurement_history.txt");
    Measurements measurement_history = measurement_parser.parse();

    // Get size of measurement history allocate memory for outputs
    size_t N = measurement_history.size();
    std::vector<State> xhat_hist; // Predicted state
    xhat_hist.reserve(N+1);
    std::vector<Covariance> xcov_hist; // Prediction covariance
    xcov_hist.reserve(N+1);
    std::vector<double> nis_hist; // Normalized innovation squared
    nis_hist.reserve(N+1);

    // Push initial states back onto output
    xhat_hist.push_back(xk_est);
    xcov_hist.push_back(xcovk_est);

    // Construct state and measurement models
    ClockStateModel f;
    ClockMeasurementModel h;
    // Construct EKF
    SimpleEKF ekf;

    // Create initial process and measurement noise
    Noise w_k = Noise::Zero();
    Noise v_kp1 = Noise::Zero();

    // Loop through all measurements
    for (const auto& y_kp1: measurement_history)
    {
        // Run iteration of EKF
        auto ekf_output = ekf({xk_est, xcovk_est},
                              {w_k, clock_noise_cov},
                              y_kp1,
                              {v_kp1, measurement_noise_cov},
                              f,
                              h);

        // Store outputs
        xhat_hist.push_back(ekf_output.state_kp1.state);
        xcov_hist.push_back(ekf_output.state_kp1.covariance);

        // Calculate and store normalized innovation squared
        const auto& Z_kp1 = ekf_output.innovation_kp1.state;
        const auto& S_kp1 = ekf_output.innovation_kp1.covariance;
        nis_hist.push_back(Z_kp1.transpose() * S_kp1 * Z_kp1);

        // Update state for next iteration
        xk_est = xhat_hist.back();
        xcovk_est = xcov_hist.back();
    }

    // Create parser for pulling in expected output data
    OutputParser output_parser(data_dir + "/filter_output.txt");
    States output = output_parser.parse();

    // Compare my output to expected output with L2 norm of state error
    for (size_t i = 0; i < output.size(); ++i)
    {
        State my_estimate = xhat_hist[i];
        State expected_output = output[i];

        // Ensure very small error
        if ((my_estimate - expected_output).norm() > 1E-10)
        {
            std::cout << "Error in expected output." << std::endl;
        }
    }
}
