#include <iostream>
#include <vector>
#include <cmath>
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
    double M_hat, C_hat, D_hat; // Estimated parameters
    double gamma_M, gamma_C, gamma_D; // Adaptation gains
    double K, alpha; // Control gains
    
    // Storing temporary states for the update_parameters step
    double current_r, current_Y_M, current_Y_C, current_Y_D;

public:
    AdaptiveModController(double M0, double C0, double D0) 
        : M_hat(M0), C_hat(C0), D_hat(D0), K(10.0), alpha(5.0), 
          gamma_M(2.0), gamma_C(2.0), gamma_D(2.0) {}

    double get_control(double x, double x_dot, double x_d, double x_d_dot, double x_d_ddot) override {
        double e = x_d - x;
        double e_dot = x_d_dot - x_dot;
        
        // Filtered tracking error components
        current_r = e_dot + alpha * e;
        double x_r_dot = x_d_dot + alpha * e;
        double x_r_ddot = x_d_ddot + alpha * e_dot;
        
        // Regression variables (Y)
        current_Y_M = x_r_ddot;
        current_Y_C = x_r_dot;
        current_Y_D = 1.0;
        
        // Control Law: u = Y * theta_hat + K * r
        double u = (current_Y_M * M_hat) + (current_Y_C * C_hat) + (current_Y_D * D_hat) + (K * current_r);
        return u;
    }

    void update_parameters(double dt) override {
        // Modular Update Law: Gradient Descent (theta_dot = Gamma * Y^T * r)
        M_hat += (gamma_M * current_Y_M * current_r) * dt;
        C_hat += (gamma_C * current_Y_C * current_r) * dt;
        D_hat += (gamma_D * current_Y_D * current_r) * dt;
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