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