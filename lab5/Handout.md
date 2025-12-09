# ROS2 Laboratory 5 – Robot Control in Simulation (MuJoCo)

In the previous lab we interfaced ROS2 with **real sensors** (USB camera, LiDAR) and learned how to capture, visualize, and process real-world data.
Up to now, our ROS2 work has focused mainly on **communication**, **parameters**, **launch files**, and **sensing**.

In this lab we take our first big step into **robot control**.

We will work with a simulated **KUKA LBR iiwa 14** robot arm inside **MuJoCo**, using two provided Python controllers:

- a **velocity-based operational-space controller**, and
- a **force/torque-based controller**.

Although these scripts are *not* ROS2 nodes, they build directly on the motion concepts you have learned and prepare you for integrating simulation, planning, and control into a ROS2 ecosystem.

# Explore the KUKA Robot Model

Navigate to the `lab5` directory:

```
cd lab5
```

This lab contains several important files:

- **iiwa14.xml** – the main KUKA robot MJCF file
- **scene.xml** – a simple scene containing the robot
- **scene_motor.xml** – a variant where joint torques are controlled
- **kuka_vel_ctrl.py** – task-space velocity controller
- **kuka_force_ctrl.py** – task-space force controller
- **assets/** – mesh files used for rendering

Open `iiwa14.xml` and `scene_motor.xml` to get familiar with the robot structure:

- joints (`<joint>`)
- links/bodies (`<body>`)
- actuators (`<actuator>`)
- sites used for control targets

> **Student TODO:** The difference between `iiwa14.xml` and `iiwa14_motor.xml` is in the actuators and how they are defined. What is the difference between the two versions of the same robot?

# Task-space velocity control

In this exercise, we will control the KUKA robot’s end-effector directly in task space (position and orientation) to make it follow a smooth circular trajectory in 3D. At each time step, the controller compares the current pose of the end-effector to a desired pose on the circle, computes a pose error, and then turns that error into a desired end-effector velocity using proportional–integral (PI) feedback. This task-space velocity command is then mapped back to joint velocities using the robot’s Jacobian, so that each joint moves in a coordinated way to correct the end-effector’s motion. We will use this velocity update rule:

$$V = V_d + K_p X_{error} + K_I \int{X_{error}}dt$$

where $V_d$ is the desired velocity in task-space and $X_{error}$ is the pose error in task space.

Overall, the goal is to see how high-level Cartesian motion objectives (like “draw a circle in space”) can be achieved by a low-level velocity controller that runs in terms of joint variables.

Open `kuka_vel_ctrl.py`.

From a high level the controller:

1. Loads the model
2. Finds the end-effector site
3. Computes position and orientation errors
4. Uses the Jacobian to compute joint velocities
5. Sends velocity commands to actuators

```python
import numpy as np
import mujoco as mj
import mujoco.viewer

def vee(R):
    """vee operator for a 3x3 skew-symmetric matrix."""
    return np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) * 0.5

def orientation_error(R_curr, R_des):
    """
    e_R = 0.5 * (R_des * R_curr^T - R_curr * R_des^T)^vee
    This is equivalent to the so(3) log for small errors and is standard in robotics.
    """
    R_err = R_des @ R_curr.T
    return vee(R_err - R_err.T)

def task_space_error(pos_curr, R_curr, pos_des, R_des):
    """
    Given current and desired position and rotation this function returns a 6D
    twist vector representing the task space error.
    """
    e_p = pos_des - pos_curr
    e_R = orientation_error(R_curr, R_des)
    return np.concatenate([e_p, e_R])   # X_e in world frame (6,)

def eight_figure(t, omega, height, R_des, b=0.5, c=0.5):
    """
    Return (pos_des, R_des, Vd) following the figure-eight trajectory:
    x(t) = b * cos(ω t) / (1 + sin^2(ω t))
    y(t) = c * sqrt(2) * sin(2 ω t) / (1 + sin^2(ω t))

    and their corresponding time derivatives.
    """

    wt  = omega * t
    s   = np.sin(wt)
    cwt = np.cos(wt)

    sin2 = np.sin(2 * wt)
    cos2 = np.cos(2 * wt)

    denom = 1 + s**2
    denom_vel = (cos2 - 3)**2

    x = b * cwt / denom
    y = c * np.sqrt(2) * sin2 / denom
    z = height

    pos_des = np.array([x, y, z])

    vx = b * (-2 * omega * s * (cos2 + 5)) / denom_vel
    vy = c * (4 * np.sqrt(2) * omega * (3 * cos2 - 1)) / denom_vel
    vz = 0.0

    v_lin = np.array([vx, vy, vz])

    v_ang = np.zeros(3)

    Vd = np.concatenate([v_lin, v_ang])

    return pos_des, R_des, Vd

def circle_traj(t, center, radius, omega, R_des):
    """
    Return (pos_des, R_des, Vd) for time t of a circle-shaped trajectory.
    """
    # Desired position on circle
    x = center[0] + radius * np.cos(omega * t)
    y = center[1] + radius * np.sin(omega * t)
    z = center[2]
    pos_des = np.array([x, y, z])

    # Desired velocity
    vx = -radius * omega * np.sin(omega * t)
    vy =  radius * omega * np.cos(omega * t)
    vz = 0.0
    v_lin = np.array([vx, vy, vz])

    # Keep orientation fixed
    v_ang = np.zeros(3)

    Vd = np.concatenate([v_lin, v_ang])
    return pos_des, R_des, Vd

def main():
    # Load robot model
    ####### ENTER CODE HERE #######

    ###############################

    # Locate end-effector site
    ee_site = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "attachment_site")

    # Simulation parameters
    ####### ENTER CODE HERE #######

    ###############################

    integral_error = np.zeros(6)

    # Define circular trajectory
    center = np.array([0.0, 0.0, 0.8])
    radius = 0.4

    # Desired velocity
    omega  = 0.15

    # Keep a fixed orientation
    R_des = np.eye(3)

    t = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:

        # Camera parameters
        viewer.cam.type = mj.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = np.array([1.0, 0.0, 1.])
        viewer.cam.distance = 2.0
        viewer.cam.azimuth = 180
        viewer.cam.elevation = -15

        # Start simulation loop

        while viewer.is_running():

            # Get desired pose and velocity at time t
            ####### ENTER CODE HERE #######

            ###############################

            # Current pose
            ####### ENTER CODE HERE #######

            ###############################

            # Task-space error
            ####### ENTER CODE HERE #######

            ###############################

            # Integral error
            integral_error += X_e * dt

            # Task-space velocity command
            ####### ENTER CODE HERE #######

            ###############################

            # Jacobian in world frame
            nv = model.nv
            jacp = np.zeros((3, nv))
            jacr = np.zeros((3, nv))
            mj.mj_jacSite(model, data, jacp, jacr, ee_site)
            J = np.vstack((jacp, jacr))

            # Joint velocities via pseudoinverse
            ####### ENTER CODE HERE #######

            ###############################
            qdot = J_pinv @ V

            # Limit joint velocities
            max_vel = 1.0  # rad/s
            v_norm = np.linalg.norm(qdot, ord=np.inf)
            if v_norm > max_vel:
                qdot *= max_vel / v_norm

            # Kinematic integration
            data.qpos[:] += qdot * dt
            ####### ENTER CODE HERE #######

            ###############################

            # Advance time forward
            ####### ENTER CODE HERE #######

            ###############################
            t += dt

            viewer.sync()

if __name__ == "__main__":
    main()
```

> **Student TODO I:** First of all we need to load our robot model and expose its data. Our model lives in the `scene.xml`.
> - Use mujoco's `MjModel.from_xml_path` function to load `scene.xml` into a variable called `model`.
> - Create a `data` variable using mujoco's `MjData` with the model as a parameter.

> **Student TODO II:** Next we need to define our simulation parameters.
> - We will set `dt` as variable with a value equal to the simulator default timestep. We can access that at `model.opt.timestep`.
> - Our `K_p` gain a diagonal 6 dimensional numpy array. The positional values are 3 and the rotational values are 2
> - Same logic for our integral `K_i` gain. The positional values are 0.1 and the rotational values are 0.05

> **Student TODO III:** We need to calculate the desired pose and velocity for this timestep 
> - Use the `circle_traj` function to get the desired position `pos_des`, desired rotation `R_des` and desired velocity `Vd`

> **Student TODO IV:** Now that we have the desired pose and velocity, we need to find our current state and calculate the error.
> - We can get the current position (`pos_curr`) of the end effector by accessing the `site_xpos` array of our `data`. Remember `site_xpos` is an array, we need to give it the proper index. In this case that is `ee_site`.
> - In a similar manner we can get the current rotation (`R_curr`) of the end effector by accessing the `site_xmat` array of our `data`. Careful `site_xmat` is an array, we need to give it the proper index. In this case that is `ee_site`. It is also flattened by default, which means it is a 9d vector instead of a 3x3 matrix. We need to use numpy to `reshape(n_rows, n_cols)`.
> - In general, when we get arrays from mujoco's `data` object, it is a good practice to copy the arrays to avoid any weird behavior. This is very simple though, we should just add the `.copy()` method to the arrays that we are getting above. 

> **Student TODO V:** Now its time to calculate our pose error.
> - Use the `task_space_error` function to get the error (`X_e`).

> **Student TODO VI:** We know the desired velocity, the controller gains and the error, which means we can calculate the task-space velocity command.
> - Calculate the task-space velocity command `V` using the equation provided above.

> **Student TODO VII:** We have the velocity command in task-space. We need to convert it to joint-level velocity command. We will the need the Jacobian pseudoinverse for that.
> - Previously in the code, we used `mj_jac_Site` to get the positional and rotational jacobians for our end effector. Now we need to calculate the pseudoinverse `J_pinv`. We can do that using numpy's `linalg.pinv` function on our full Jacobian.

> **Student TODO VIII:** Now that we have our velocity command in joint space, we need to give this information to our `data` variable and then make mujoco do forward kinematics with this new information so that our scene is updated.
> - We can see how we updated the joint positions by accessing `data.qpos` and integrating our velocity command. In a similar manner we need to update the joint velocities by accessing `data.qvel`
> - Finally we can update our scene by calling mujoco's `mj_forward` function and giving it our `model` and `data` as parameters.

# Task-space force control

In the previous exercise we controlled the robot purely at the kinematic level, directly updating joint positions based on desired end-effector motion without considering masses, forces, or the robot’s true dynamics. This is useful for understanding geometric relationships, but it does not reflect how real robots move. In this exercise we shift to fully dynamic, torque-level control, where the controller interacts with the physics engine and must respect inertia, gravity, and joint dynamics.

We'll implement an task-space force controller for the KUKA arm that makes the end-effector move along a smooth circular trajectory. Instead of directly commanding joint velocities, the controller computes forces in task-space based on position and orientation errors and their rates (like a 3D mass–spring–damper system around the desired pose):

$$F = M_x (K_p X_{error} +K_d V_{error})$$

where $M_x$ is the task-space inertia matrix, $X_{error}$ is the pose error and $V_{error}$ is the velocity error in task space. We can then map these into joint torques using the Jacobian:

$$\tau = J^T F$$

On top of this task-space behavior we need to add gravity compensation so the robot can “float” naturally and a nullspace controller that gently pulls the joints toward a comfortable posture without disturbing the end-effector motion:

$$\tau_{total} = \tau + \tau_{gravity} + \tau_{nullspace}$$

Overall, the goal is to see how desired end-effector behavior can be achieved through low-level torque control that respects the robot’s full dynamics.

Open `kuka_force_ctrl.py`.

```python
import mujoco as mj
import mujoco.viewer
import numpy as np

def orientation_error(R, R_d):
    """
    Calculate the orientation error between two rotations
    """
    R_err = R.T @ R_d
    skew = 0.5 * (R_err - R_err.T)
    err_body =  np.array([skew[2,1], skew[0,2], skew[1,0]])
    return R @ err_body

def circle_traj(t, center, radius, omega, z0, x0, t_start, T_ramp):
    """
    Return x_d and v_d for time t of a circle-shaped trajectory.
    """
    xd = np.array([
        center[0] + radius * np.cos(omega * t),
        center[1] + radius * np.sin(omega * t),
        z0
    ])
    xdot_d = np.array([
        -radius * omega * np.sin(omega * t),
        radius * omega * np.cos(omega * t),
        0.0
    ])

    # Enter the trajectory smoothly from its center
    tau = (t - t_start) / T_ramp
    tau = np.clip(tau, 0.0, 1.0)
    s = 0.5 * (1.0 - np.cos(np.pi * tau))

    xd = (1.0 - s) * x0 + s * xd
    xdot_d = s * xdot_d

    return xd, xdot_d

def main():
    # Load robot model
    ####### ENTER CODE HERE #######

    ###############################

    # End effector-site
    ee_name = "attachment_site"   
    ee_id   = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, ee_name)

    # Controller gains
    ####### ENTER CODE HERE #######

    ###############################

    Kp = np.diag([Kp_pos, Kp_pos, Kp_pos,
                Kp_rot, Kp_rot, Kp_rot])
    Kd = np.diag([Kd_pos, Kd_pos, Kd_pos,
                Kd_rot, Kd_rot, Kd_rot])


    # Nullspace controller reference trajectory
    qmin = model.jnt_range[:, 0].copy()
    qmax = model.jnt_range[:, 1].copy()

    ####### ENTER CODE HERE #######

    ###############################


    # Nullspace controller gains
    Kp_null = 5.

    damping_ratio = 1.

    Kd_null = damping_ratio * 2. * np.sqrt(Kp_null)

    M_inv = np.zeros((model.nv, model.nv))

    # Set desired orientation and circle center from initial pose

    mj.mj_forward(model, data)
    R_d = data.site_xmat[ee_id].reshape(3, 3).copy()
    x0 = data.site_xpos[ee_id].copy()

    # Define circle parameters

    center = np.array([0.0, 0.0, 0.8])
    radius = 0.4
    omega = 0.1  # rad/s
    z0 = center[2]

    # Ramp time parameters
    T_ramp = 3.0
    t_start = data.time


    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        # Camera parameters
        viewer.cam.type = mj.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = np.array([1., 0., 1.])
        viewer.cam.distance = 2.
        viewer.cam.azimuth = 180
        viewer.cam.elevation = -15

        # Simulation loop

        while viewer.is_running():
            t = data.time

            # Desired task-space quantities
            ####### ENTER CODE HERE #######

            ###############################
            omega_d = np.zeros(3)

            # Current task-space state
            x = data.site_xpos[ee_id].copy()
            R = data.site_xmat[ee_id].reshape(3, 3).copy()

            # Calculate Jacobian
            Jp = np.zeros((3, model.nv))
            Jr = np.zeros((3, model.nv))
            mj.mj_jacSite(model, data, Jp, Jr, ee_id)
            J = np.vstack([Jp, Jr])

            # Current velocities
            ####### ENTER CODE HERE #######

            ###############################

            # Errors
            ####### ENTER CODE HERE #######

            ###############################

            error_x  = np.concatenate([e_pos, e_rot])
            error_v = np.concatenate([ed_pos, ed_rot])

            # Calculate the task-space inertia matrix
            mujoco.mj_solveM(model, data, M_inv, np.eye(model.nv))
            Mx_inv = J @ M_inv @ J.T
            if abs(np.linalg.det(Mx_inv)) >= 1e-2:
                Mx = np.linalg.inv(Mx_inv)
            else:
                Mx = np.linalg.pinv(Mx_inv, rcond=1e-2)
            
            # Calculate the task space forces

            ####### ENTER CODE HERE #######

            ###############################

            # Derive joint level torques

            tau = J.T @ F_task

            # Calculate gravity compensation

            ####### ENTER CODE HERE #######

            ###############################
            
            # Add gravity term

            tau_total = tau + tau_gravity

            # Nullspace control

            Jbar = M_inv @ J.T @ Mx
            ddq = Kp_null * (q_ref - data.qpos.copy()) - Kd_null * data.qvel

            tau_nullspace = (np.eye(model.nv) - J.T @ Jbar.T) @ ddq

            # Add nullspace term
            ####### ENTER CODE HERE #######

            ###############################

            data.ctrl[:] = tau_total

            # Move the simulation forward
            ####### ENTER CODE HERE #######

            ###############################
            viewer.sync()

if __name__ == "__main__":
    main()
```

> **Student TODO I:** First of all we need to load our robot model and expose its data. Our model lives in the `scene_motor.xml`.
> - Use mujoco's `MjModel.from_xml_path` function to load `scene_motor.xml` into a variable called `model`.
> - Create a `data` variable using mujoco's `MjData` with the model as a parameter.

> **Student TODO II:** We will now define our controller gains. In this exercise we use a PD controller, so we need to define four values: `Kp` for our positional and rotational part of the error and `Kd` for our positional and rotational part of the error.
> - We will set `Kp_pos` and `Kp_rot` equal to 5.
> - `Kd_pos` and `Kd_rot` will be equal to two times the square root of `Kp_pos` and `Kp_rot` respectively.

> **Student TODO III:** We must set a reference trajectory for our nullspace controller. This will help us avoid singularities by applying additional force to the joints without moving the end effector.
> - We will set our `q_ref` at the middle point of our joint range (defined by `q_min` and `q_max`).

> **Student TODO IV:** At the start of every iteration of the control loop, we need to get our desired pose and velocity.
> - Use the `circle_traj` function to get the desired pose `xd` and the desired velocity `xdot_d`

> **Student TODO V:** We can see that once again we access `data.site_xpos` and `data.site_xmat` to get our current position and rotation in task space. In this exercise though, we also need the current velocity in task space. With mujoco we can access the joint space velocities and convert them to task space with the Jacobian.
> - We can get the current joint-space velocity (`qd`) by accessing `data.qvel`. Remember it is better to use `.copy()` on the arrays that we get from mujoco.
> - Now we can get the linear velocity part `xdot` by left multiplying with the positional Jacobian `Jp` and the angular velocity part `omega_ee` by left multiplying with the rotational jacobian `Jr`.

> **Student TODO VI:** Now that we have both desired and current pose and velocties, we can calculate the errors that our controller will use.
> - Calculate the positional part of the pose error `e_pos` by subtracting our current position from our desired position.
> - Calculate the rotational part of the pose error `e_rot` by using the function `orientation_error`.
> - Calculate the positional part of the velocity error by subtracting our current linear velocity from our desired linear velocity.
> - Calculate the rotational part of the velocity error by subtracting our current angular velocity from our desired angular velocity.

> **Student TODO VII:** You can see we have already calculated the task-space inertia matrix `Mx`. This means that we have everything to calculate the task-space force command.
> - Calculate the task-space force `F_task` by using the equation provided above.

> **Student TODO VIII:** In order to compensate for gravity and avoid singularities, we need to add the appropriate terms to our joint level torques `tau`.
> - We can get the gravity compensation torques in joint space by accessing the `qfrc_bias` field of our `data`. We will store this in a variable called `tau_gravity`.
> - We will also add our nullspace controller addition to the torques to avoid singularities. That is the `tau_nullspace`.

> **Student TODO IX:** Finally we adavance the simulation step to see what happens!
> - We need to call the mujoco's `mj_step` function and give it our `model` and `data` as parameters.