import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# --- True Plant / Physics Parameters ---
M_true = 0.2
C_true = 6.0
D_true = 1.4

# --- The Simulation Engine ---
def simulate(K, alpha, beta, k_n, gamma_M, gamma_C, gamma_D):
    dt = 0.001
    t_max = 10.0
    n_steps = int(t_max / dt)
    
    t_data = np.linspace(0, t_max, n_steps)
    x_data = np.zeros(n_steps)
    x_d_data = np.zeros(n_steps)
    
    # Initial conditions
    x = 0.0
    x_dot = 0.0
    theta_hat = np.array([0.0, 0.0, 0.0]) # [M_hat, C_hat, D_hat]
    Y_f = np.array([0.0, 0.0, 0.0])       # Low-pass filtered regressor
    Gamma = np.diag([gamma_M, gamma_C, gamma_D])
    
    for i in range(n_steps):
        t = t_data[i]
        
        # 1. Desired Trajectory
        x_d = np.sin(2*t)
        x_d_dot = np.cos(2*t)
        x_d_ddot = -np.sin(2*t)
        
        # 2. Error Metrics
        e = x_d - x
        e_dot = x_d_dot - x_dot
        r = e_dot + alpha * e
        
        x_r_dot = x_d_dot + alpha * e
        x_r_ddot = x_d_ddot + alpha * e_dot
        
        # 3. Regressor Vectors
        Y = np.array([x_r_ddot, x_r_dot, 1.0])
        
        # 4. Parameter Update Rates (Modularity Core)
        theta_dot = Gamma @ Y * r
        M_hat_dot = theta_dot[0]
        
        # 5. Robust Modular Control Law (Equation 12)
        term1 = K * r
        term2 = np.dot(Y, theta_hat)
        term3 = (1.0 / beta) * np.dot(Y_f, theta_dot)
        term4 = M_hat_dot * r
        term5 = k_n * np.sum(Y**2) * r
        term6 = k_n * ((1.0 / beta) * np.dot(Y_f, theta_dot))**2 * r
        term7 = k_n * (M_hat_dot * r)**2 * r
        
        u = term1 + term2 + term3 + term4 + term5 + term6 + term7
        
        # 6. Physical System Dynamics (The Plant)
        x_ddot = (u - C_true * x_dot - D_true) / M_true
        x_dot += x_ddot * dt
        x += x_dot * dt
        
        # 7. Update Adaptive Parameters (Euler Integration)
        theta_hat += theta_dot * dt
        Y_f += beta * (Y - Y_f) * dt
        
        # Store Data
        x_data[i] = x
        x_d_data[i] = x_d
        
    return t_data, x_data, x_d_data

# --- UI Setup ---
fig, ax = plt.subplots(figsize=(10, 7))
plt.subplots_adjust(left=0.1, bottom=0.4) # Leave room for sliders at bottom

# Initial Simulation Run
init_K, init_alpha, init_beta, init_kn, init_gamma = 10.0, 5.0, 10.0, 1.0, 2.0
t, x, x_d = simulate(init_K, init_alpha, init_beta, init_kn, init_gamma, init_gamma, init_gamma)

line_x, = ax.plot(t, x, label='Actual Position (x)', color='blue')
line_xd, = ax.plot(t, x_d, '--', label='Desired Position (x_d)', color='orange')
ax.legend(loc='upper right')
ax.set_title("Interactive Adaptive Control Tuning")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Position")
ax.grid(True)
ax.set_ylim(-2, 2) # Fixed y-axis for consistent viewing

# --- Sliders ---
axcolor = 'lightgoldenrodyellow'
ax_K     = plt.axes([0.15, 0.30, 0.65, 0.03], facecolor=axcolor)
ax_alpha = plt.axes([0.15, 0.25, 0.65, 0.03], facecolor=axcolor)
ax_beta  = plt.axes([0.15, 0.20, 0.65, 0.03], facecolor=axcolor)
ax_kn    = plt.axes([0.15, 0.15, 0.65, 0.03], facecolor=axcolor)
ax_gam   = plt.axes([0.15, 0.10, 0.65, 0.03], facecolor=axcolor)

# Slider limits (Name, Min, Max, Initial)
s_K     = Slider(ax_K,     'K (Gain)',     0.0, 50.0, valinit=init_K)
s_alpha = Slider(ax_alpha, 'Alpha',       0.1, 20.0, valinit=init_alpha)
s_beta  = Slider(ax_beta,  'Beta (Filter)',1.0, 50.0, valinit=init_beta)
s_kn    = Slider(ax_kn,    'k_n (Damping)',0.0, 5.0,  valinit=init_kn)
s_gam   = Slider(ax_gam,   'Gamma (Learn)',0.1, 20.0, valinit=init_gamma)

# --- The "On Change" Event ---
def update(val):
    # Recalculate simulation with new slider values
    t_new, x_new, x_d_new = simulate(s_K.val, s_alpha.val, s_beta.val, s_kn.val, s_gam.val, s_gam.val, s_gam.val)
    
    # Update the graph data
    line_x.set_ydata(x_new)
    
    # Dynamically adjust y-limits if it goes unstable
    ax.set_ylim(min(-2.0, np.min(x_new) - 0.5), max(2.0, np.max(x_new) + 0.5))
    fig.canvas.draw_idle()

# Bind sliders to update function
s_K.on_changed(update)
s_alpha.on_changed(update)
s_beta.on_changed(update)
s_kn.on_changed(update)
s_gam.on_changed(update)

plt.show()