import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
class Robot:
    def __init__(self, x, y, z, vx, vy, vz, v_max,v_min, a_max,a_min, dt, alpha=0.3, beta=0.7, gamma=0.1):
        """
        初始化机器人状态和参数

        Args:
            x, y, z: 初始位置
            vx, vy, vz: 初始速度
            v_max: 最大线速度
            a_max: 最大加速度
            dt: 时间步长
            alpha: 目标距离的权重
            beta: 障碍物距离的权重
            gamma: 速度偏好的权重
        """
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.v_max = v_max  # 最大线速度
        self.v_min = v_min  # 最小线速度
        self.a_max = a_max  # 最大线加速度
        self.a_min = a_min  # 最小线加速度
        self.dt = dt  # 时间步长

        # 权重参数
        self.alpha = alpha  # 目标距离权重
        self.beta = beta  # 障碍物距离权重
        self.gamma = gamma  # 速度偏好权重

    def update_state(self, new_position, new_velocity):
        """
        更新机器人的位置和速度

        Args:
            new_position: 新位置 (x, y, z)
            new_velocity: 新速度 (vx, vy, vz)
        """
        self.x, self.y, self.z = new_position
        self.vx, self.vy, self.vz = new_velocity

    def get_state(self):
        """
        获取当前机器人状态

        Returns:
            当前状态: (x, y, z, vx, vy, vz)
        """
        return (self.x, self.y, self.z, self.vx, self.vy, self.vz)

    def set_weights(self, alpha, beta, gamma):
        """
        动态调整权重参数

        Args:
            alpha: 目标距离的权重
            beta: 障碍物距离的权重
            gamma: 速度偏好的权重
        """
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma

    def get_weights(self):
        """
        获取当前权重参数

        Returns:
            权重参数: (alpha, beta, gamma)
        """
        return (self.alpha, self.beta, self.gamma)
class MovingGoal:
    def __init__(self, x, y, z, bounds, v_max=1.0, direction_change_interval=10):
        """
        Initialize a structured random moving goal.

        Args:
            x, y, z: Initial position.
            bounds: Movement boundaries (x_min, x_max, y_min, y_max, z_min, z_max).
            v_max: Maximum velocity.
            direction_change_interval: Steps after which direction is recalculated.
        """
        self.x = x
        self.y = y
        self.z = z
        self.bounds = bounds
        self.v_max = v_max
        self.direction_change_interval = direction_change_interval
        self.step_count = 0

        # Initialize random direction
        self.vx, self.vy, self.vz = self._generate_random_direction()

    def _generate_random_direction(self):
        """
        Generate a random direction vector with fixed magnitude.
        """
        direction = np.random.randn(3)  # Random vector
        direction = direction / np.linalg.norm(direction) * self.v_max  # Normalize and scale
        return direction[0], direction[1], direction[2]

    def update_position(self, dt):
        """
        Update position using a structured random movement.

        Args:
            dt: Time step.
        """
        # Update position
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        # Keep within bounds and bounce back if hitting boundaries
        if not (self.bounds[0] <= self.x <= self.bounds[1]):
            self.vx *= -1
            self.x = np.clip(self.x, self.bounds[0], self.bounds[1])
        if not (self.bounds[2] <= self.y <= self.bounds[3]):
            self.vy *= -1
            self.y = np.clip(self.y, self.bounds[2], self.bounds[3])
        if not (self.bounds[4] <= self.z <= self.bounds[5]):
            self.vz *= -1
            self.z = np.clip(self.z, self.bounds[4], self.bounds[5])

        # Change direction periodically
        self.step_count += 1
        if self.step_count >= self.direction_change_interval:
            self.vx, self.vy, self.vz = self._generate_random_direction()
            self.step_count = 0

    def get_position(self):
        """
        Get the current position of the goal.

        Returns:
            (x, y, z): Current position of the goal.
        """
        return (self.x, self.y, self.z)
class DynamicObstacle:
    def __init__(self, x, y, z, vx, vy, vz, bounds, v_max=0.5, direction_change_interval=50):
        """
        Initialize a dynamic obstacle with velocity and bounds.

        Args:
            x, y, z: Initial position.
            vx, vy, vz: Initial velocity.
            bounds: Movement boundaries (x_min, x_max, y_min, y_max, z_min, z_max).
            v_max: Maximum velocity.
            direction_change_interval: Steps after which direction is recalculated.
        """
        self.size_range=(1, 3) #
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.size = np.random.uniform(self.size_range[0], self.size_range[1])    # 随机大小
        self.bounds = bounds
        self.v_max = v_max
        self.direction_change_interval = direction_change_interval
        self.step_count = 0

    def _generate_random_direction(self):
        """Generate a random direction vector with fixed magnitude."""
        direction = np.random.randn(3)  # Random vector
        direction = direction / np.linalg.norm(direction) * self.v_max  # Normalize and scale
        return direction[0], direction[1], direction[2]

    def update_position(self, dt):
        """
        Update position using a structured random movement.

        Args:
            dt: Time step.
        """
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        # Bounce back on boundary collision
        if not (self.bounds[0] <= self.x <= self.bounds[1]):
            self.vx *= -1
            self.x = np.clip(self.x, self.bounds[0], self.bounds[1])
        if not (self.bounds[2] <= self.y <= self.bounds[3]):
            self.vy *= -1
            self.y = np.clip(self.y, self.bounds[2], self.bounds[3])
        if not (self.bounds[4] <= self.z <= self.bounds[5]):
            self.vz *= -1
            self.z = np.clip(self.z, self.bounds[4], self.bounds[5])

        # Periodically change direction
        self.step_count += 1
        if self.step_count >= self.direction_change_interval:
            self.vx, self.vy, self.vz = self._generate_random_direction()
            self.step_count = 0

    def get_position(self):
        """Return the current position and radius of the obstacle."""
        return ('sphere', self.x, self.y, self.z, self.size, None)  # Assuming dynamic obstacles are spheres
# Generate mixed obstacles
def generate_mixed_obstacles(num_obstacles, space_size, radius_range, height_range):
    obstacles = []
    for _ in range(num_obstacles):
        obstacle_type = np.random.choice(['sphere', 'cylinder'])
        x = np.random.uniform(0, space_size)
        y = np.random.uniform(0, space_size)
        if obstacle_type == 'sphere':
            z = np.random.uniform(0, space_size)  # Spheres can float
            radius = np.random.uniform(radius_range[0], radius_range[1])
            obstacles.append(('sphere', x, y, z, radius, None))
        else:  # Cylinder
            z = 0  # Cylinders start at ground level
            radius = np.random.uniform(radius_range[0], radius_range[1])
            height = np.random.uniform(height_range[0], height_range[1])
            obstacles.append(('cylinder', x, y, z, radius, height))
    return obstacles
# Predict trajectories
def predict_trajectory(robot, vx, vy, vz):
    trajectory = []
    x, y, z = robot.x, robot.y, robot.z
    for _ in range(5):  # Predict for 10 steps
        x += vx * robot.dt
        y += vy * robot.dt
        z += vz * robot.dt
        trajectory.append((x, y, z))
    return np.array(trajectory)

def evaluate_trajectory(trajectory, goal, obstacles, robot, safe_distance):
    """
    评价给定轨迹的质量，综合考虑目标距离、障碍物安全性和轨迹时间效率。
    
    Args:
        trajectory: UAV的预测轨迹，形状为 (N, 3)。
        goal: 目标点坐标 (x, y, z)。
        obstacles: 障碍物列表，每个障碍物包含 (类型, ox, oy, oz, radius, height)。
        robot: Robot对象，包含评价权重等参数。
        safe_distance: 安全距离，若小于该值则显著降低得分。

    Returns:
        轨迹得分：值越高越优。
    """
    # 权重系数（可以从Robot类中动态调整）
    alpha = robot.alpha  # 目标距离的权重
    beta = robot.beta    # 障碍物距离的权重
    gamma = robot.gamma  # 速度偏好的权重

    # 初始化评价值
    # goal_distance = np.linalg.norm(trajectory[-1] - np.array(goal))  # 目标距离
    min_obstacle_distance = float('inf')  # 障碍物最小距离

    # # 计算轨迹的平均速度
    # total_distance = np.sum(np.linalg.norm(np.diff(trajectory, axis=0), axis=1))
    # average_speed = total_distance / (len(trajectory) * robot.dt*5)
    # 计算轨迹总距离
    total_distance = np.sum(np.linalg.norm(np.diff(trajectory, axis=0), axis=1))

    # 计算轨迹的总时间
    total_time = (len(trajectory) - 1) * robot.dt

    # 计算平均速度
    average_speed = total_distance / total_time

    # 裁剪平均速度，确保在 [v_min, v_max] 范围内
    average_speed = np.clip(average_speed, robot.v_min, robot.v_max)

    # 归一化速度得分
    speed_score = (average_speed - robot.v_min) / (robot.v_max - robot.v_min)


        # 初始化最小距离
    min_obstacle_distance = float('inf')

    # 遍历轨迹每个点，计算与障碍物的最小距离
    for point in trajectory:
        for obs in obstacles:
            obs_type, ox, oy, oz, radius, height = obs
            if obs_type == 'sphere':
                # 球体障碍物距离计算
                dist = np.linalg.norm(point - np.array([ox, oy, oz])) - radius
            elif obs_type == 'cylinder':
                # 圆柱体障碍物距离计算
                horizontal_dist = np.linalg.norm(point[:2] - np.array([ox, oy])) - radius
                if 0 <= point[2] <= height:
                    dist = horizontal_dist
                else:
                    dist = float('inf')  # 超出高度范围
            else:
                dist = float('inf')  # 未知障碍物类型
            
            # 更新最小距离
            min_obstacle_distance = min(min_obstacle_distance, dist)

    # 根据最小距离计算评分
    if min_obstacle_distance < 0:
        # 在障碍物内部，得分为负数
        obstacle_score = min_obstacle_distance
    elif min_obstacle_distance < safe_distance:
        # 在障碍物外部，但距离小于安全距离
        obstacle_score = min_obstacle_distance / safe_distance
    else:
        # 距离超过安全距离，满分
        obstacle_score = 1.0

        # 计算初始点到目标的最大距离
    max_distance = np.linalg.norm(trajectory[0] - np.array(goal))



    # 起点和终点
    start_point = trajectory[0]  # 起点
    goal_point = np.array(goal)  # 目标点

    # 计算起点到目标点的方向向量
    direction_to_goal = (goal_point - start_point) / np.linalg.norm(goal_point - start_point)

    # 计算在给定时间内可以到达的最远点
    d_max = robot.v_max * robot.dt  # 最大可移动距离
    reachable_point = start_point + direction_to_goal * d_max

    # 计算最远点与目标点的距离
    min_goal_distance = np.linalg.norm(reachable_point - goal_point)

    # 计算轨迹终点与目标点的实际距离
    goal_distance = np.linalg.norm(trajectory[-1] - goal_point)

    # 计算目标得分（归一化）
    goal_score =  min_goal_distance / (goal_distance + 1e-6)  # 防止除零


    # speed_score = average_speed-robot.v_min / (robot.v_max - robot.v_min)  # 归一化速度得分

    # 综合得分（越高越优）
    total_score = alpha * goal_score + beta * obstacle_score + gamma * speed_score
    return total_score

def cal_dynamic_window_vel( state, obstacle):
    """速度采样,得到速度空间窗口 (3D)

    Args:
        v (list): 当前速度 [vx, vy, vz,]
        state (list): 当前机器人状态 [x, y, z,  vx, vy, vz,]
        obstacle (np.ndarray): 障碍物位置，形状为 (N, 3)
        
    Returns:
        list: 最终采样后的速度空间 [vx_low, vx_high, vy_low, vy_high, vz_low, vz_high]
    """
    # 计算速度边界限制

    # 计算三种限制
    Vm = __cal_vel_limit(state)  # 速度边界限制
    Vd = __cal_accel_limit(state)  # 加速度限制
    Va = __cal_obstacle_limit(state, obstacle)  # 障碍物限制

    # 取三种限制的交集范围
    vx_low = max(Vm[0], Vd[0], Va[0])
    vx_high = min(Vm[1], Vd[1], Va[1])
    vy_low = max(Vm[2], Vd[2], Va[2])
    vy_high = min(Vm[3], Vd[3], Va[3])
    vz_low = max(Vm[4], Vd[4], Va[4])
    vz_high = min(Vm[5], Vd[5], Va[5])
  

    return [vx_low, vx_high, vy_low, vy_high, vz_low, vz_high]

def __cal_vel_limit(state):
    """计算速度边界限制 Vm

    Returns:
        list: 速度边界限制 [vx_min, vx_max, vy_min, vy_max, vz_min, vz_max, omega_min, omega_max]
    """
    return [state.v_min, state.v_max, state.v_min, state.v_max, state.v_min, state.v_max]

def __cal_accel_limit(state):
    """计算加速度限制 Vd

    Args:
        vx (float): 当前 x 方向速度
        vy (float): 当前 y 方向速度
        vz (float): 当前 z 方向速度
        omega_z (float): 当前绕 z 轴角速度
        
    Returns:
        list: 考虑加速度时的速度空间 Vd
    """
    vx_low = state.vx - state.a_max * state.dt
    vx_high = state.vx + state.a_max * state.dt
    vy_low = state.vy - state.a_max * state.dt
    vy_high = state.vy + state.a_max * state.dt
    vz_low = state.vz - state.a_max * state.dt    
    vz_high = state.vz + state.a_max * state.dt

    return [vx_low, vx_high, vy_low, vy_high, vz_low, vz_high]

def __cal_obstacle_limit(state, combined_obstacles):
    """环境障碍物限制 Va

    Args:
        state (list): 当前机器人状态 [x, y, z, yaw, vx, vy, vz, omega_z]
        combined_obstacles (list): 混合障碍物列表，包括静态和动态障碍物。
                                   静态障碍物为 (type, x, y, z, radius, height)
                                   动态障碍物为 (type, x, y, z, radius, height)

    Returns:
        list: 考虑障碍物限制的速度空间 Va
    """
    # 提取当前机器人位置
    x, y, z = state.x, state.y, state.z
    # 初始化最近距离为一个较大的值
    min_dist = float('inf')

    # 遍历所有障碍物，找到最近距离
    for obs in combined_obstacles:
        obs_type, obs_x, obs_y, obs_z, radius, height = obs

        if obs_type == 'sphere':
            # 计算与球形障碍物的几何距离
            dist = np.sqrt((x - obs_x)**2 + (y - obs_y)**2 + (z - obs_z)**2) - radius

        elif obs_type == 'cylinder':
            # 计算与圆柱障碍物的几何距离
            xy_dist = np.sqrt((x - obs_x)**2 + (y - obs_y)**2) - radius
            z_dist = max(0, abs(z - obs_z) - height / 2)  # Z方向距离
            dist = max(xy_dist, z_dist)

        # 更新最小距离
        if dist < min_dist:
            min_dist = dist

    # 根据最近障碍物距离限制速度
    vx_low = state.v_min
    vx_high = min(state.v_max, np.sqrt(2 * max(min_dist, 0) * state.a_max))
    vy_low = state.v_min
    vy_high = min(state.v_max, np.sqrt(2 * max(min_dist, 0) * state.a_max))
    vz_low = state.v_min
    vz_high = min(state.v_max, np.sqrt(2 * max(min_dist, 0) * state.a_max))
   

    return [vx_low, vx_high, vy_low, vy_high, vz_low, vz_high]

# Dynamic Window Approach
def dynamic_window_approach(robot, goal, obstacles, safe_distance):
    best_trajectory = None
    best_score = 0
    best_velocity = (0, 0, 0)
    [vx_low, vx_high, vy_low, vy_high, vz_low, vz_high]=cal_dynamic_window_vel(robot,obstacles)

    # Sample velocities
    for vx in np.linspace(vx_low, vx_high, 20):
        for vy in np.linspace(vy_low, vy_high, 20):
            for vz in np.linspace(vz_low, vz_high, 10):
                trajectory = predict_trajectory(robot, vx, vy, vz)
                # trajectory=interpolate_trajectory(trajectory, 50)
                score = evaluate_trajectory(trajectory, goal, obstacles,robot, safe_distance)
                if score >best_score:  # 更新最优轨迹
                    best_score = score
                    best_trajectory = trajectory
                    best_velocity = (vx, vy, vz)

    return best_trajectory, best_velocity

def generate_dynamic_obstacles(num_obstacles, space_size, radius_range, velocity_range):
    dynamic_obstacles = []
    for _ in range(num_obstacles):
        x = np.random.uniform(0, space_size)
        y = np.random.uniform(0, space_size)
        z = np.random.uniform(0, space_size)
        radius = np.random.uniform(radius_range[0], radius_range[1])
        vx = np.random.uniform(velocity_range[0], velocity_range[1])
        vy = np.random.uniform(velocity_range[0], velocity_range[1])
        vz = np.random.uniform(velocity_range[0], velocity_range[1])
        bounds = (0, space_size, 0, space_size, 0, space_size)
        dynamic_obstacles.append(DynamicObstacle(x, y, z, radius, vx, vy, vz, bounds))
    return dynamic_obstacles

def plot_quadrotor(ax, x, y, z, yaw=0, size=1.0, arm_length=1.0, rotor_radius=0.2, motor_size=0.1, color='blue'):
    uav_elements = []  # To store all plot elements

    # Define arm endpoints relative to the center
    arm_offsets = np.array([
        [arm_length, 0, 0],  # Arm 1
        [-arm_length, 0, 0],  # Arm 2
        [0, arm_length, 0],  # Arm 3
        [0, -arm_length, 0],  # Arm 4
    ]) * size

    # Rotate arms by the yaw angle
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    rotated_offsets = arm_offsets @ rotation_matrix.T

    # Translate arms to the quadrotor's position
    arm_positions = rotated_offsets + np.array([x, y, z])

    # Draw the arms
    for i in range(4):
        arm_plot, = ax.plot(
            [x, arm_positions[i, 0]],  # X-coordinates
            [y, arm_positions[i, 1]],  # Y-coordinates
            [z, arm_positions[i, 2]],  # Z-coordinates
            color=color, linewidth=2
        )
        uav_elements.append(arm_plot)

    # Precompute rotor surface grid
    u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
    rotor_x_base = rotor_radius * np.cos(u) * np.sin(v)
    rotor_y_base = rotor_radius * np.sin(u) * np.sin(v)
    rotor_z_base = rotor_radius * np.cos(v)

    # Draw the motors as spheres and rotors as circles
    for pos in arm_positions:
        # Draw motor
        motor = _plot_sphere(ax, pos[0], pos[1], pos[2], radius=motor_size, color='black', u=u, v=v)
        uav_elements.append(motor)

        # Draw rotor
        rotor_surface = ax.plot_surface(
            pos[0] + rotor_x_base, pos[1] + rotor_y_base, pos[2] + rotor_z_base,
            color='grey', alpha=0.6
        )
        uav_elements.append(rotor_surface)

    return uav_elements

def _plot_sphere(ax, x, y, z, radius=0.1, color='black', u=None, v=None):
    if u is None or v is None:
        u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
    xs = x + radius * np.cos(u) * np.sin(v)
    ys = y + radius * np.sin(u) * np.sin(v)
    zs = z + radius * np.cos(v)
    return ax.plot_surface(xs, ys, zs, color=color, alpha=0.9)

def simulate_with_obstacles(robot, goal, static_obstacles, dynamic_obstacles, safe_distance, space_size, steps=100):
    # Combine static and dynamic obstacles
    combined_obstacles = static_obstacles + [obs.get_position() for obs in dynamic_obstacles]

    # Continue the simulation logic as before
    trajectories = []

    # Set up the 3D plot
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0, space_size)
    ax.set_ylim(0, space_size)
    ax.set_zlim(0, space_size)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("DWA Simulation with Static and Dynamic Obstacles")

    # Plot goal
    goal_plot = ax.scatter(*goal.get_position(), color='g', s=100, label='Moving Goal')

    # Plot static obstacles
    for obs in static_obstacles:
        obs_type, x, y, z, radius, height = obs  # Unpack all 6 values
        if obs_type == 'sphere':
            # Plot a sphere
            u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
            xs = x + radius * np.cos(u) * np.sin(v)
            ys = y + radius * np.sin(u) * np.sin(v)
            zs = z + radius * np.cos(v)
            ax.plot_surface(xs, ys, zs, color='r', alpha=0.6)
        elif obs_type == 'cylinder':
            # Plot a cylinder
            z_cyl = np.linspace(0, height, 20)
            theta = np.linspace(0, 2 * np.pi, 30)
            theta_grid, z_grid = np.meshgrid(theta, z_cyl)
            x_cyl = x + radius * np.cos(theta_grid)
            y_cyl = y + radius * np.sin(theta_grid)
            ax.plot_surface(x_cyl, y_cyl, z_grid, color='r', alpha=0.6)


    dynamic_obstacle_plots = []
    for obstacle in dynamic_obstacles:
        # Use scatter to plot dynamic obstacles with sizes proportional to their radii
        _,x, y, z, radius, _ = obstacle.get_position()
        obstacle_plot = ax.scatter(x, y, z, s=radius * 100, c='blue', label='Dynamic Obstacle', alpha=0.7)  # Scale radius
        dynamic_obstacle_plots.append(obstacle_plot)


    # Plot UAV trajectory and current position
    trajectory_plot, = ax.plot([], [], [], '-k', label='UAV Trajectory')
    current_position_plot, = ax.plot([], [], [], 'ro', label='Current Position', markersize=5)
    uav_elements = []

    def update(frame):
        nonlocal robot, trajectories,uav_elements

        for i, obstacle in enumerate(dynamic_obstacles):
                obstacle.update_position(robot.dt)
                _, x, y, z, radius, _ = obstacle.get_position()  # Get updated position and radius
                # Update scatter plot position and size
                dynamic_obstacle_plots[i]._offsets3d = ([x], [y], [z])
                dynamic_obstacle_plots[i]._sizes = [radius * 100]  # Scale radius for visualization

        # Combine static and dynamic obstacles for collision checking
        combined_obstacles = static_obstacles + [obs.get_position() for obs in dynamic_obstacles]
        # Remove previous UAV plot
        # 移除旧元素
        for elem in uav_elements:
            if elem in ax.collections or elem in ax.lines:
                elem.remove()

        uav_elements = []  # Clear references
        # Check if goal is reached
        if np.linalg.norm(np.array(robot.get_state()[:3]) - np.array(goal.get_position())) < safe_distance:
            print("Goal reached!")
            # Plot UAV at the final position
            uav_elements = plot_quadrotor(ax, robot.x, robot.y, robot.z, yaw=np.pi / 6, size=0.5, arm_length=0.75, rotor_radius=0.15, motor_size=0.075)
            return trajectory_plot, current_position_plot, *dynamic_obstacle_plots, *uav_elements
            # return trajectory_plot, current_position_plot, *dynamic_obstacle_plots

        # Use DWA to find the best trajectory
        trajectory, velocity = dynamic_window_approach(robot, goal.get_position(), combined_obstacles, safe_distance)
        if trajectory is None:
            print("No valid trajectory found! UAV stopped.")
            return trajectory_plot, current_position_plot, *dynamic_obstacle_plots

        # Append trajectory and update robot state
        trajectories.append(trajectory)
        robot.update_state(trajectory[1], velocity)

        # Combine all past trajectories for plotting
        full_trajectory = np.concatenate(trajectories, axis=0)

        # Update trajectory plot
        trajectory_plot.set_data(full_trajectory[:, 0], full_trajectory[:, 1])
        trajectory_plot.set_3d_properties(full_trajectory[:, 2])
        # # Plot UAV at the new position
        uav_elements = plot_quadrotor(ax, robot.x, robot.y, robot.z, yaw=np.pi / 6, size=0.5, arm_length=0.75, rotor_radius=0.15, motor_size=0.075)
         # Update current position plot
        current_position_plot.set_data([robot.x], [robot.y])
        current_position_plot.set_3d_properties([robot.z])
        # Update goal position
        goal.update_position(robot.dt)
        goal_position = goal.get_position()
        goal_plot._offsets3d = ([goal_position[0]], [goal_position[1]], [goal_position[2]])

        return trajectory_plot, current_position_plot, *dynamic_obstacle_plots
    
    # Run animation
    anim = FuncAnimation(fig, update, frames=steps, interval=50, blit=False)

    plt.legend()
    plt.show()

    return trajectories




if __name__=="__main__":
    # Define space boundaries (x_min, x_max, y_min, y_max, z_min, z_max)
    space_size = 20
    bounds = (0, space_size, 0, space_size, 0, space_size)

    # Create the UAV (robot)
    robot = Robot(x=1, y=1, z=10, vx=0, vy=0, vz=0, v_max=1.0, v_min=-1.5,a_max=1.0, a_min=-0.5, dt=0.1)

    # Create the moving goal
    goal = MovingGoal(x=18, y=18, z=18, bounds=bounds, v_max=0, direction_change_interval=50)

    # Define static obstacles with consistent format
    # Define static obstacles
    static_obstacles = [
        # ('sphere', 5, 5, 5, 1.5, None),  # Sphere at (5, 5, 5) with radius 1.5
        # ('sphere', 10, 10, 10, 2.0, None),  # Sphere at (10, 10, 10) with radius 2.0
        ('cylinder', 15, 15, 0, 1.0, 20.0) ,  # Cylinder at (15, 15) with radius 1.0 and height 5.0
        ('cylinder', 3, 3, 0, 1.0,20.0),  # Cylinder at (2, 2) with radius 1.0 and height 10.0
        ('cylinder', 6, 15, 0, 1.0, 20.0) ,  # Cylinder at (15, 15) with radius 1.0 and height 5.0
        # ('cylinder', 10, 3, 0, 1.0,20.0) , # Cylinder at (2, 2) with radius 1.0 and height 10.0
        # ('cylinder', 7, 3, 0, 1.5,20.0)  ,# Cylinder at (2, 2) with radius 1.0 and height 10.0

        # ('cylinder', 5, 10, 0, 1.5,20.0),  # Cylinder at (2, 2) with radius 1.0 and height 10.0
        # ('cylinder', 10, 15, 0, 1.5,20.0)  # Cylinder at (2, 2) with radius 1.0 and height 10.0


        ]


    # Create dynamic obstacles
    dynamic_obstacles = [
        DynamicObstacle(x=18, y=18, z=18, vx=0.2, vy=0.2, vz=0.1, bounds=bounds, v_max=5)
        # DynamicObstacle(x=15, y=16, z=18, vx=-0.1, vy=0.2, vz=0.15, bounds=bounds, v_max=5)
    ]

    # Define the safety distance
    safe_distance = 1

    # Run the simulation
    trajectories = simulate_with_obstacles(robot, goal, static_obstacles, dynamic_obstacles, safe_distance, space_size, steps=200)

    # Visualize results
    print("Simulation completed.")