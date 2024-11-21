# 3D Dynamic Window Approach (DWA) Simulation with Obstacles

## Overview
This project implements a **3D Dynamic Window Approach (DWA)** simulation for a quadrotor UAV navigating in a space with static and dynamic obstacles. The UAV dynamically plans its trajectory to reach a moving goal while avoiding collisions with obstacles.

The simulation is visualized in 3D using **Matplotlib**, showcasing the UAV's real-time trajectory, moving goal, and obstacles.

---

## Features
1. **3D Dynamic Window Approach (DWA):**  
   - Predicts UAV trajectories based on velocity samples.
   - Evaluates trajectories considering goal distance, obstacle safety, and velocity efficiency.

2. **Static and Dynamic Obstacles:**  
   - Static obstacles include spheres and cylinders.  
   - Dynamic obstacles have random structured movement within defined bounds.

3. **Moving Goal:**  
   - The target moves within boundaries with a random but structured trajectory.

4. **Real-Time 3D Visualization:**  
   - Displays UAV, goal, and obstacles.
   - Animates UAV trajectory in a 3D space.

---

## Project Structure
### Classes
1. **`Robot`:**  
   Represents the UAV with dynamic parameters such as position, velocity, and motion constraints.
   
2. **`MovingGoal`:**  
   A randomly moving goal that updates its position periodically within defined bounds.

3. **`DynamicObstacle`:**  
   A dynamic obstacle with random direction changes and velocity within bounds.

### Core Functions
1. **`dynamic_window_approach(robot, goal, obstacles, safe_distance)`:**  
   Implements the DWA algorithm to find the optimal trajectory for the UAV.

2. **`evaluate_trajectory(trajectory, goal, obstacles, robot, safe_distance)`:**  
   Evaluates a trajectory using a weighted scoring system considering:
   - Distance to the goal.
   - Distance to obstacles.
   - Velocity preference.

3. **`simulate_with_obstacles(robot, goal, static_obstacles, dynamic_obstacles, safe_distance, space_size, steps)`:**  
   Simulates the UAV's movement using DWA and visualizes the results in 3D.

4. **`plot_quadrotor`:**  
   Renders a 3D quadrotor UAV in the simulation plot.

5. **Obstacle Management Functions:**
   - **`generate_mixed_obstacles`:** Creates a mixture of static sphere and cylinder obstacles.
   - **`generate_dynamic_obstacles`:** Creates a list of moving obstacles with random properties.

---

## Requirements
### Libraries
- **Python 3.x**
- **NumPy**
- **Matplotlib**

To install the required libraries, run:
```bash
pip install numpy matplotlib
```

---

## Usage
1. **Run the Script**  
   Execute the script in any Python IDE or terminal:
   ```bash
   python drone_dwa.py
   ```

2. **Customize Parameters**  
   Adjust the following parameters in the main section to modify the simulation:
   - **Space Size:**  
     ```python
     space_size = 20
     ```
   - **Robot Properties:**  
     ```python
     robot = Robot(x=1, y=1, z=10, vx=0, vy=0, vz=0, v_max=1.0, v_min=-1.5, a_max=1.0, a_min=-0.5, dt=0.1)
     ```
   - **Goal Properties:**  
     ```python
     goal = MovingGoal(x=18, y=18, z=18, bounds=bounds, v_max=0.5, direction_change_interval=50)
     ```
   - **Static Obstacles:**  
     ```python
     static_obstacles = [
         ('cylinder', 15, 15, 0, 1.0, 20.0),
         ('sphere', 10, 10, 10, 2.0, None)
     ]
     ```
   - **Dynamic Obstacles:**  
     ```python
     dynamic_obstacles = [
         DynamicObstacle(x=18, y=18, z=18, vx=0.2, vy=0.2, vz=0.1, bounds=bounds, v_max=5)
     ]
     ```
   - **Simulation Steps:**  
     ```python
     steps = 200
     ```

3. **Simulation Output:**  
   Observe the UAV's trajectory in the 3D visualization.

---

## Example
The script initializes a UAV at position `(1, 1, 10)` with no initial velocity. The goal moves within the bounds `(0, 20)` and the UAV avoids static cylinders and a dynamic obstacle while navigating toward the goal.

---

## Customization
You can extend this simulation by:
- Adding more sophisticated obstacle movement patterns.
- Modifying the UAV's dynamics or adding constraints.
- Changing the visualization style for better clarity.

---

## Future Work
- Integrate advanced UAV dynamics with realistic aerodynamics.
- Add more complex environments with terrain and weather effects.
- Implement reinforcement learning for obstacle avoidance.

---

## License
This project is open-source and available for educational and research purposes.