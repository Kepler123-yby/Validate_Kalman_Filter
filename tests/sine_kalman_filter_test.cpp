#include "tools/sine_fuction/sine_kalman_filter.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <random>

int main()
{
    const tools::SineFunction truth{2.2, 1.35, 0.45, -0.3};

    tools::SineKalmanFilterConfig config;
    config.initial_model = {1.6, 1.2, 0.1, 0.0};
    config.initial_covariance =
        (Eigen::Vector4d() << 1.0, 0.2, 1.0, 0.5).finished().asDiagonal();
    config.process_noise << 1e-5, 1e-7, 1e-5, 1e-5;
    config.measurement_variance = 0.04 * 0.04;
    config.innovation_gate_sigma = 0.0;

    tools::SineKalmanFilter filter(config);
    std::mt19937 random_engine(42);
    std::normal_distribution<double> noise(0.0, 0.04);

    constexpr double dt = 0.01;
    constexpr int sample_count = 4000;
    double squared_error = 0.0;
    int error_count = 0;

    for (int i = 0; i < sample_count; ++i)
    {
        const double t = i * dt;
        const double measurement = truth.evaluate(t) + noise(random_engine);
        assert(filter.update(t, measurement));

        if (i >= sample_count / 2)
        {
            const double error = filter.evaluate(t) - truth.evaluate(t);
            squared_error += error * error;
            ++error_count;
        }
    }

    const auto fitted = filter.model();
    const double rmse = std::sqrt(squared_error / error_count);

    std::cout << "truth:  A=" << truth.A << " f=" << truth.f
              << " phi=" << truth.phi << " x=" << truth.x << '\n';
    std::cout << "fitted: A=" << fitted.A << " f=" << fitted.f
              << " phi=" << fitted.phi << " x=" << fitted.x << '\n';
    std::cout << "steady-state RMSE=" << rmse << '\n';

    assert(std::abs(fitted.f - truth.f) < 0.03);
    assert(std::abs(std::abs(fitted.A) - truth.A) < 0.08);
    assert(std::abs(fitted.x - truth.x) < 0.05);
    assert(rmse < 0.08);
    return 0;
}
