# README：基于动态窗口法的3D障碍物避障仿真

## 项目概述
本项目实现了一个基于**动态窗口法（Dynamic Window Approach, DWA）**的3D无人机（UAV）避障仿真系统。无人机通过动态规划轨迹，在存在静态和动态障碍物的环境中实时避障，目标是到达一个不断移动的目标点。

仿真通过 **Matplotlib** 进行3D可视化，动态展示无人机的运动轨迹、移动目标以及障碍物的变化。

---

## 功能特点
1. **动态窗口法 (DWA) 实现：**
   - 采样多种速度轨迹并预测未来位置。
   - 根据目标距离、障碍物安全性和速度效率综合评估轨迹。

2. **静态与动态障碍物：**
   - 支持球体和圆柱体两种静态障碍物。
   - 动态障碍物在边界内随机移动。

3. **移动目标：**
   - 目标点按照一定规律随机移动，并动态更新位置。

4. **实时3D可视化：**
   - 实时显示无人机、目标点和障碍物的位置及变化。
   - 动态更新无人机的运动轨迹。

---

## 项目结构
### 类说明
1. **`Robot`：**  
   表示无人机，包含位置、速度、动态约束等参数。

2. **`MovingGoal`：**  
   一个随机移动的目标点，周期性更新其位置。

3. **`DynamicObstacle`：**  
   表示动态障碍物，具有随机方向和速度。

### 核心函数
1. **`dynamic_window_approach(robot, goal, obstacles, safe_distance)`：**  
   实现动态窗口法，为无人机选择最佳轨迹。

2. **`evaluate_trajectory(trajectory, goal, obstacles, robot, safe_distance)`：**  
   评价轨迹得分，综合考虑：
   - 与目标的距离。
   - 障碍物安全性。
   - 速度偏好。

3. **`simulate_with_obstacles(robot, goal, static_obstacles, dynamic_obstacles, safe_distance, space_size, steps)`：**  
   执行仿真，并在3D环境中动态展示无人机轨迹。

4. **`plot_quadrotor`：**  
   绘制无人机的3D模型。

5. **障碍物管理函数：**
   - **`generate_mixed_obstacles`：** 随机生成球体和圆柱体静态障碍物。
   - **`generate_dynamic_obstacles`：** 创建具有随机属性的动态障碍物。

---

## 运行环境
### 所需库
- **Python 3.x**
- **NumPy**
- **Matplotlib**

通过以下命令安装所需库：
```bash
pip install numpy matplotlib
```

---

## 使用方法
1. **运行脚本：**  
   在任意Python IDE或终端中运行脚本：
   ```bash
   python drone_dwa.py
   ```

2. **修改参数：**  
   根据需要调整主函数中的以下参数：
   - **环境大小：**
     ```python
     space_size = 20
     ```
   - **无人机属性：**
     ```python
     robot = Robot(x=1, y=1, z=10, vx=0, vy=0, vz=0, v_max=1.0, v_min=-1.5, a_max=1.0, a_min=-0.5, dt=0.1)
     ```
   - **目标属性：**
     ```python
     goal = MovingGoal(x=18, y=18, z=18, bounds=bounds, v_max=0.5, direction_change_interval=50)
     ```
   - **静态障碍物：**
     ```python
     static_obstacles = [
         ('cylinder', 15, 15, 0, 1.0, 20.0),
         ('sphere', 10, 10, 10, 2.0, None)
     ]
     ```
   - **动态障碍物：**
     ```python
     dynamic_obstacles = [
         DynamicObstacle(x=18, y=18, z=18, vx=0.2, vy=0.2, vz=0.1, bounds=bounds, v_max=5)
     ]
     ```
   - **仿真步数：**
     ```python
     steps = 200
     ```

3. **观察仿真结果：**  
   运行脚本后，观察3D可视化中无人机的运动轨迹及障碍物变化。

---

## 示例
默认情况下，脚本初始化了一个无人机，起始位置为 `(1, 1, 10)`，速度为零。目标点在 `(18, 18, 18)` 内随机移动，无人机需避开静态圆柱体和一个动态障碍物，并尝试到达目标。

---

## 定制化
本项目支持以下扩展：
- 添加更复杂的障碍物运动模式。
- 修改无人机的动力学模型或增加更多约束。
- 改进可视化风格以更清晰地展示数据。

---



---

## 开源协议
本项目为开源项目，供教育和科研用途自由使用。