# Adaptive Control of Robot Manipulators

This project implements an Object-Oriented C++ simulation of a 1-DOF robotic system ($M\ddot{x} + C\dot{x} + D = u$) using a modular adaptive control law. It is inspired by the modularity concepts discussed in the paper *"Adaptive control of robot manipulators with controller/update law modularity"* by de Queiroz et al.

The simulation calculates the required control inputs, updates adaptive parameter estimates in real-time, and visualizes the system's actual vs. desired trajectories using native C++ bindings for Python's Matplotlib (`matplotlibcpp.h`).

## Prerequisites

To build and run this project, you need the following dependencies installed on your system:

1. **C++ Compiler**: A compiler that supports C++17 (e.g., Apple Clang, GCC).
2. **CMake**: Version 3.14 or higher.
3. **Python 3**: Python 3.x along with its development headers.
4. **Python Packages**: `numpy` and `matplotlib` for rendering the simulation graphics.

### Installing Dependencies

If you haven't already, you can install the required Python libraries using `pip`:

```bash
python3 -m pip install numpy matplotlib
```

**Note for macOS:** CMake attempts to find your Python installation automatically. If you have multiple versions of Python installed (like macOS default Python vs. Homebrew Python), CMake might get confused. If `cmake ..` fails to find NumPy, open `CMakeLists.txt` and update the `Python3_EXECUTABLE` path to point directly to the python interpreter where you installed `numpy` (`which python3`).

## Building the Project

This project uses **CMake** to generate the build files, which keeps cross-platform linking (like tying C++ to Python) completely automated and clean.

Follow these steps from the root of the project directory:

1. **Create and navigate to a build directory**:
   This keeps all compiled files separate from your source code.

   ```bash
   mkdir build
   cd build
   ```
2. **Generate the Makefiles**:

   ```bash
   cmake ..
   ```
3. **Compile the executable**:

   ```bash
   make
   ```

## Running the Simulation

Once the project is successfully compiled, you can launch the simulation directly from inside the `build` directory:

```bash
./neuro_sim
```

**What to expect:**

- The internal C++ physics engine will simulate 10 seconds of the adaptive controller trying to track a desired sine wave trajectory.
- A Matplotlib UI window will appear, graphing both the *Actual Position* curve and the *Desired Position* curve.
- A high-resolution copy of the graph will automatically be saved as `tracking_plot.png`.
