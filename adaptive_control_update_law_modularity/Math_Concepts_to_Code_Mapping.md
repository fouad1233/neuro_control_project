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

## 4. The Control Law

### The Theory
The total control signal $u$ sent to the robot is the sum of the adaptive feedforward term (the robot's believed dynamics) and a proportional feedback term to pull $r$ to zero:
$$ u = Y \hat{\theta} + K r $$

### The Code
Thanks to the Eigen library, the C++ code performs this matrix multiplication cleanly on one line:
```cpp
// Control Law (Vectorized): u = Y * theta_hat + K * r
double u = (current_Y * theta_hat).value() + (K * current_r);
```

---

## 5. The Adaptation (Update) Law

### The Theory
How does the controller learn the unknown mass, friction, and gravity? By gradient descent on the error! The update law defines how the parameter estimates $\hat{\theta}$ change over time based on the Regressor $Y$, the error $r$, and a learning rate matrix $\Gamma$:
$$ \dot{\hat{\theta}} = \Gamma Y^T r $$

### The Code
To run this continuous differential equation on a computer, we use Euler integration (multiplying the derivative by the time-step `dt` and accumulating it). 
*   `Gamma` is a 3x3 diagonal matrix for tuning the learning rates individually.
*   `current_Y.transpose()` turns the row vector back into a column vector so the matrix dimensions align perfectly.

```cpp
void update_parameters(double dt) override {
    // Modular Update Law Vectorized: theta_dot = Gamma * Y^T * r
    theta_hat += Gamma * current_Y.transpose() * current_r * dt;
}
```