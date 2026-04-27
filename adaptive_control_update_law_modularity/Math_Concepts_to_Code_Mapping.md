# Adaptive Control: Mathematical Theory and Code Implementation

This document breaks down the mathematical concepts from the adaptive control literature (specifically based on Slotine and Li's adaptive manipulator control) and maps them directly to the 1-Degree-of-Freedom (1D) C++ simulation in `main.cpp`.

## 1. System Dynamics

### The Theory
A general rigid-body robot manipulator is governed by the following dynamic equation:
$$ M(q)\ddot{q} + V_m(q, \dot{q})\dot{q} + G(q) + F_d\dot{q} = u $$
Where:
*   $M(q)$ is the inertia matrix.
*   $V_m(q, \dot{q})$ represents Coriolis and Centripetal forces.
*   $G(q)$ is gravity.
*   $F_d$ is the viscous friction coefficient.

**In a 1D System:**
*   $M$ becomes a constant scalar mass. 
*   **Coriolis and Centripetal forces ($V_m$) are exactly strictly zero.** Why? These are interactive forces that require multiple moving joints. Mathematically, $V_m$ depends on the spatial derivative of the inertia matrix ($\frac{\partial M}{\partial q}$). Since $M$ is a constant block of mass in 1D, its derivative is zero, meaning $V_m = 0$.
*   $F_d$ translates simply to a linear damping/friction constant $C$.
*   $G(q)$ translates to a constant disturbance or gravity $D$.

### The Code
This simplifies the dynamics to $M\ddot{x} + C\dot{x} + D = u$, which is exactly how the `Plant` class computes acceleration physically:
```cpp
class Plant {
    // Solves M*x_ddot + C*x_dot + D = u for x_ddot
    double compute_acceleration(double x_dot, double u) {
        return (u - C * x_dot - D) / M;
    }
};
```

---

## 2. Reference Kinematics and the Sliding Surface
To ensure stability, adaptive controllers track a "reference" trajectory combined with the error, rather than just raw desired states. This creates a combined tracking error metric often called $r$ or $s$.

### The Theory
*   **Position Error:** $e = q_d - q$
*   **Sliding Surface / Filtered Error:** $r = \dot{e} + \alpha e$
*   **Reference Velocity:** $\dot{q}_r = \dot{q}_d + \alpha e$
*   **Reference Acceleration:** $\ddot{q}_r = \ddot{q}_d + \alpha \dot{e}$

### The Code
This is computed at the beginning of the `get_control` method:
```cpp
double e = x_d - x;
double e_dot = x_d_dot - x_dot;

current_r = e_dot + alpha * e;
double x_r_dot = x_d_dot + alpha * e;
double x_r_ddot = x_d_ddot + alpha * e_dot;
```

---

## 3. Linear Parameterization (The Regressor Matrix $Y$)

### The Theory
Adaptive controllers rely on **Linear Parameterization**. This means we can factor out the unknown parameters ($\theta$) from the known robot states (like velocity and acceleration) using a Regressor Matrix ($Y$). 

From Equation (11) in the literature, the parameterization form is:
$$ Y_s(q, \dot{q}, t)\theta = M(q)(\ddot{q}_d + \alpha\dot{e}) + V_m(q, \dot{q})(\dot{q}_d + \alpha e) + G(q) + F_d\dot{q} $$

As established, for our 1D simulation:
*   $M(q)(\ddot{q}_d + \alpha\dot{e})$ becomes $M \ddot{x}_r$
*   $V_m(q, \dot{q})$ is $0$, so that entire term vanishes.
*   $F_d\dot{q}$ maps to $C \dot{x}_r$.
*   $G(q)$ maps to constant $D$ multiplied by $1$.

So we can separate the variables like a dot product into a row vector (Regressor $Y$) and a column vector (Parameters $\theta$):
$$ Y \theta = \begin{bmatrix} \ddot{x}_r & \dot{x}_r & 1 \end{bmatrix} \begin{bmatrix} M \\ C \\ D \end{bmatrix} = M\ddot{x}_r + C\dot{x}_r + D $$

### The Code
We store the unknown parameters in a vertically-aligned Eigen vector `theta_hat`, and construct the row-vector `current_Y` exactly as the math demands:
```cpp
// Regression vector (Y)
current_Y << x_r_ddot, x_r_dot, 1.0;
```
The `1.0` serves as the placeholder to multiply against the gravity/disturbance estimate $\hat{D}$.

---

## 4. The Robust Modular Control Law (Equation 12)

### The Theory
Instead of the standard classic tracking law ($u = Y\hat{\theta} + Kr$), this implementation uses the **Robust Modular Control Law** from Equation 12. This advanced equation mathematically decouples the control law from the parameter update law, allowing for safer, faster parameter tuning while injecting "nonlinear damping" to prevent instability during aggressive learning.

The formula from the literature is:
$$ \tau = Kr + Y_s\hat{\theta} + \left[\frac{1}{\beta}Y_f\dot{\hat{\theta}}\right] + [(\dot{\hat{M}} - \hat{V}_m)r] + k_n\|Y_s\|^2 r + k_n\left\|\frac{1}{\beta}Y_f\dot{\hat{\theta}}\right\|^2 r + k_n\|(\dot{\hat{M}}-\hat{V}_m)r\|^2 r $$

Because this is a 1D system, $\hat{V}_m = 0$. The equation breaks down as:
1. **$Kr$**: Baseline proportional/derivative feedback.
2. **$Y_s\hat{\theta}$**: Baseline adaptive feedforward.
3. **$\frac{1}{\beta}Y_f\dot{\hat{\theta}}$**: Filtered modularity compensator (uses a low-pass filtered regressor $Y_f$).
4. **$\dot{\hat{M}}r$**: Kinetic energy compensator (counters wildly fluctuating mass estimates).
5. **$k_n \| \cdot \|^2 r$ terms**: "Nonlinear damping" terms. These mathematically act as brakes. If the norm (magnitude) of the regressors or adaptations gets too large, these square-law terms dominate the control signal to forcefully stabilize the robot.

### The Code
The C++ code explicitly calculates $\dot{\hat{\theta}}$ first, then splits Equation 12 into discrete terms for clarity:
```cpp
// Compute current theta_dot (from baseline update law)
Eigen::Vector3d theta_dot = Gamma * current_Y.transpose() * current_r;
double M_hat_dot = theta_dot(0); // V_m_hat is 0 in 1D

// Equation 12: Robust Modular Control Law
double term1 = K * current_r;                                             // Kr
double term2 = (current_Y * theta_hat).value();                           // Y_s * theta_hat
double term3 = (1.0 / beta) * (Y_f * theta_dot).value();                  // Filtered update force
double term4 = M_hat_dot * current_r;                                     // (M_hat_dot - V_m_hat)*r
double term5 = k_n * current_Y.squaredNorm() * current_r;                 // k_n * ||Y_s||^2 * r
double term6 = k_n * std::pow(term3, 2) * current_r;                      // k_n * ||1/beta * Y_f * theta_dot||^2 * r
double term7 = k_n * std::pow(M_hat_dot * current_r, 2) * current_r;      // k_n * ||(M_hat_dot)*r||^2 * r

double u = term1 + term2 + term3 + term4 + term5 + term6 + term7;
```

---

## 5. The Adaptation Law and Regressor Filter

### The Theory
To make the modular control law work, two differential equations must be solved simultaneously alongside the physical simulation:
1. **The Parameter Update ($\dot{\hat{\theta}}$)**: Gradient descent to learn the unknown parameters based on the error $r$.
   $$ \dot{\hat{\theta}} = \Gamma Y_s^T r $$
2. **The Regressor Filter ($\dot{Y}_f$)**: A low-pass filter applied to the regression matrix to smooth out high-frequency mathematical noise during parameter updates, governed by bandwidth $\beta$.
   $$ \dot{Y}_f = \beta(Y_s - Y_f) $$

### The Code
We use Euler integration (multiplying the derivatives by the time-step `dt`) to step both equations forward in time:
```cpp
void update_parameters(double dt) override {
    // 1. Modular Update Law
    Eigen::Vector3d theta_dot = Gamma * current_Y.transpose() * current_r;
    theta_hat += theta_dot * dt;
    
    // 2. Update low-pass filtered regression matrix (Y_f)
    Eigen::RowVector3d Y_f_dot = beta * (current_Y - Y_f);
    Y_f += Y_f_dot * dt;
}
```