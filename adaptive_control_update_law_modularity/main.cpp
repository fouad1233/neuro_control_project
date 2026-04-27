#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Interface for the Controller
class Controller {
public:
    virtual ~Controller() = default;
    
    // The controller receives states and desired states, returning the control signal 'u'
    virtual double get_control(double x, double x_dot, double x_d, double x_d_dot, double x_d_ddot) = 0;
    
    // Updates internal estimated parameters (the "update law" modularity)
    virtual void update_parameters(double dt) = 0;
};

class Plant {
private:
    double M, C, D;
public:
    Plant(double m, double c, double d) : M(m), C(c), D(d) {}

    // Solves M*x_ddot + C*x_dot + D = u for x_ddot
    double compute_acceleration(double x_dot, double u) {
        return (u - C * x_dot - D) / M;
    }
};

class AdaptiveModController : public Controller {
private:
    Eigen::Vector3d theta_hat; // Estimated parameters vector: [M_hat, C_hat, D_hat]^T
    Eigen::Matrix3d Gamma;     // Adaptation gain matrix [gamma_M,0,0; 0, gamma_C, 0; 0, 0, gamma_D]
    double K, alpha;           // Control gains
    
    // Variables for Modular/Robust Control (Equation 12)
    Eigen::RowVector3d Y_f;    // Low-pass filtered regressor
    double beta;               // Filter bandwidth
    double k_n;                // Nonlinear damping gain
    
    // Storing temporary states for the update_parameters step
    double current_r;
    Eigen::RowVector3d current_Y; // Regression row vector Y = [x_r_ddot, x_r_dot, 1] for the current time step

public:
    AdaptiveModController(double M0, double C0, double D0) 
        : K(10.0), alpha(5.0), beta(10.0), k_n(1.0) { // Added robust parameters
        
        // Initialize parameter vector
        theta_hat << M0, C0, D0;
        
        // Initialize diagonal adaptation gain matrix
        Gamma = Eigen::Matrix3d::Zero();
        Gamma.diagonal() << 2.0, 2.0, 2.0;  // Gains for M, C, D respectively
        
        // Initialize Filtered Regressor to zero
        Y_f = Eigen::RowVector3d::Zero();
    }

    double get_control(double x, double x_dot, double x_d, double x_d_dot, double x_d_ddot) override {
        double e = x_d - x;
        double e_dot = x_d_dot - x_dot;
        
        // Filtered tracking error components
        current_r = e_dot + alpha * e;
        double x_r_dot = x_d_dot + alpha * e;
        double x_r_ddot = x_d_ddot + alpha * e_dot;
        
        // Regression vector (Y)
        current_Y << x_r_ddot, x_r_dot, 1.0;
        
        // Compute current theta_dot (from baseline update law)
        Eigen::Vector3d theta_dot = Gamma * current_Y.transpose() * current_r;
        double M_hat_dot = theta_dot(0); // V_m_hat is 0 in 1D
        
        // Equation 12: Robust Modular Control Law
        double term1 = K * current_r;
        double term2 = (current_Y * theta_hat).value();
        double term3 = (1.0 / beta) * (Y_f * theta_dot).value();
        double term4 = M_hat_dot * current_r;                                     // (M_hat_dot - V_m_hat) * r
        double term5 = k_n * current_Y.squaredNorm() * current_r;                 // k_n * ||Y_s||^2 * r
        double term6 = k_n * std::pow(term3, 2) * current_r;                      // k_n * ||1/beta * Y_f * theta_dot||^2 * r
        double term7 = k_n * std::pow(M_hat_dot * current_r, 2) * current_r;      // k_n * ||(M_hat_dot - 0)*r||^2 * r
        
        double u = term1 + term2 + term3 + term4 + term5 + term6 + term7;
        return u; // Scalar output
    }

    void update_parameters(double dt) override {
        // Modular Update Law Vectorized: theta_dot = Gamma * Y_s^T * r
        Eigen::Vector3d theta_dot = Gamma * current_Y.transpose() * current_r;
        theta_hat += theta_dot * dt;
        
        // Update low-pass filtered regression matrix (Y_f)
        // filter dynamics: Y_f_dot = beta * (Y_s - Y_f)
        Eigen::RowVector3d Y_f_dot = beta * (current_Y - Y_f);
        Y_f += Y_f_dot * dt;
    }
};

int main() {
    Plant sys(0.2, 6.0, 1.4); // True parameters
    AdaptiveModController ctrl(0.0, 0.0, 0.0); // Start with 0 knowledge
    
    double x = 0.0, x_dot = 0.0; // Initial conditions
    double dt = 0.001; 
    
    std::vector<double> time_data, x_data, x_d_data;
    
    for (double t = 0; t <= 10.0; t += dt) {
        // 1. Define desired trajectory (e.g., Sine wave)
        double x_d = std::sin(t); 
        double x_d_dot = std::cos(t);
        double x_d_ddot = -std::sin(t);
        
        // 2. Compute control input
        double u = ctrl.get_control(x, x_dot, x_d, x_d_dot, x_d_ddot);
        
        // 3. System physics update (Euler integration)
        double x_ddot = sys.compute_acceleration(x_dot, u);
        x_dot += x_ddot * dt;
        x += x_dot * dt;
        
        // 4. Update adaptive parameters
        ctrl.update_parameters(dt);
        
        // Save data for plotting
        time_data.push_back(t);
        x_data.push_back(x);
        x_d_data.push_back(x_d);
    }

    // --- Plotting using matplotlibcpp ---
    plt::figure();
    plt::named_plot("Actual Position (x)", time_data, x_data);
    plt::named_plot("Desired Position (x_d)", time_data, x_d_data, "--");
    
    plt::title("Adaptive Control Tracking result");
    plt::xlabel("Time [s]");
    plt::ylabel("Position");
    plt::legend();
    
    // Automatically save the image and then display it
    plt::save("tracking_plot.png"); 
    plt::show(); 

    return 0;
}