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