# """CrazyFlie software-in-the-loop control example.


# Setup
# -----
# Step 1: Clone pycffirmware from [https://github.com/utiasDSL/pycffirmware](https://github.com/utiasDSL/pycffirmware)
# Step 2: Follow the install instructions for pycffirmware in its README


# Example
# -------
# In terminal, run:
# python gym_pybullet_drones/examples/cf.py --seed 42


# """


###
# Logging is added!
###
# gym-pybullet-drones\gym_pybullet_drones\envs\CrtlAviary_prey_preds.py


import os
import sys


current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)


import time
import argparse
import stat
import numpy as np
from utils.enums import DroneModel, Physics
from envs.CrtlAviary_prey_preds import CtrlAviary_
from control.DSLPIDControl import DSLPIDControl
from utils.utils import sync, str2bool
from flocking import FlockingUtils
import pybullet as p
from datetime import datetime


import re
import os
import sys




# CHANGE HERE
BOUNDLESS = True
if BOUNDLESS:
    NS = 10
    PERC_NO_SENSOR = 0#0.6
else:
    NS = 5
    PERC_NO_SENSOR = 0#0.6
_3D = False
PLOT = True
self_log = False
GUI = True
#############
DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_GUI = GUI
DEFAULT_USER_DEBUG_GUI = GUI
DEFAULT_PLOT = False
DEFAULT_SEED = None  # Add default seed parameter


DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_OUTPUT_FOLDER = 'results'
DURATION_SEC = 30  # Reduced from 200 to 30 seconds





NUM_DRONES = NS
PREYS = NS
if_cube = True
drones_ids = list(range(NUM_DRONES))


def initialize_positions_with_seed(seed=None):
    """Initialize positions with optional random seed for reproducibility."""
    if seed is not None:
        np.random.seed(seed)
        print(f"Random seed set to: {seed}")
    
    if _3D:
        min_distance = 3.
    else:
        if BOUNDLESS:       #sigma_prey                 #sesnor_range - sigma_predator   
            # min_distance = (1.0 * np.sqrt(PREYS/2)) + ( 3 - 2.0 + PERC_NO_SENSOR ) # Minimum distance between predators and preys
            # min_distance = ((np.sqrt(PREYS) * 1.0 * np.sqrt(2))/(2*np.pi))+3
            min_distance = (np.sqrt(PREYS) * 1.7 * 0.3) + 3/1.5 # Minimum distance between predators and preys
            # min_distance = 2.5
        else: #LAB
            min_distance = 1.3


    if BOUNDLESS:
        init_center_x = 3
        init_center_y = 3
        init_center_z = 3
        init_center_x_preys = init_center_x + np.random.choice([-1, 1]) * min_distance#
        init_center_y_preys = init_center_y + np.random.choice([-1, 1]) * min_distance#
        init_center_z_preys = init_center_z + 0
        spacing = 0.95
    else:
        init_center_x = 1#4.5
        init_center_y = 1#2.5
        init_center_z = 2
        init_center_x_preys = init_center_x + min_distance
        init_center_y_preys = init_center_y +  min_distance
        init_center_z_preys = init_center_z + 0
        spacing = 0.2

    f_util = FlockingUtils(NUM_DRONES,
                           PREYS, 
                           init_center_x, 
                           init_center_y, 
                           init_center_z,
                           spacing, 
                           init_center_x_preys, 
                           init_center_y_preys, 
                           init_center_z_preys, 
                           drones_ids=drones_ids, 
                           perc_no_sensor=PERC_NO_SENSOR,
                           boundless=BOUNDLESS,
                           _3D=_3D)

    if _3D:
        pos_xs, pos_ys, pos_zs, pos_hxs, pos_hys, pos_hzs = f_util.initialize_positions3D(preds=True)
        pos_xs_preys, pos_ys_preys, pos_zs_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys = f_util.initialize_positions3D(preds=False)
    else:
        pos_xs, pos_ys, pos_zs, pos_h_xc, pos_h_yc, pos_h_zc = f_util.initialize_positions(preds=True)
        pos_xs_preys, pos_ys_preys, pos_zs_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys = f_util.initialize_positions(preds=False)

    return f_util, pos_xs, pos_ys, pos_zs, pos_xs_preys, pos_ys_preys, pos_zs_preys


def run(
        drone=DEFAULT_DRONES,
        num_drones=NUM_DRONES,
        preys=PREYS,
        physics=DEFAULT_PHYSICS,
        gui=DEFAULT_GUI,
        plot=DEFAULT_PLOT,
        user_debug_gui=DEFAULT_USER_DEBUG_GUI,
        simulation_freq_hz=DEFAULT_SIMULATION_FREQ_HZ,
        control_freq_hz=DEFAULT_CONTROL_FREQ_HZ,
        output_folder=DEFAULT_OUTPUT_FOLDER,
        duration_sec=DURATION_SEC,
        seed=DEFAULT_SEED  # Add seed parameter to run function
):
    # Initialize positions with seed
    f_util, pos_xs, pos_ys, pos_zs, pos_xs_preys, pos_ys_preys, pos_zs_preys = initialize_positions_with_seed(seed)

    INIT_XYZ = np.zeros([NUM_DRONES, 3])
    INIT_XYZ[:, 0] = pos_xs
    INIT_XYZ[:, 1] = pos_ys
    INIT_XYZ[:, 2] = pos_zs
    INIT_RPY = np.array([[.0, .0, .0] for _ in range(NUM_DRONES)])

    INIT_XYZ_PREYS = np.zeros([PREYS, 3])
    INIT_XYZ_PREYS[:, 0] = pos_xs_preys
    INIT_XYZ_PREYS[:, 1] = pos_ys_preys
    INIT_XYZ_PREYS[:, 2] = pos_zs_preys
    INIT_RPY_PREYS = np.array([[.0, .0, .0] for _ in range(PREYS)])

    env = CtrlAviary_(drone_model=ARGS.drone,
                     num_drones=ARGS.num_drones,
                     preys=ARGS.preys,
                     initial_xyzs=INIT_XYZ,
                     initial_rpys=INIT_RPY,
                     initial_xyzs_preys=INIT_XYZ_PREYS,
                     initial_rpys_preys=INIT_RPY_PREYS,
                     physics=ARGS.physics,
                     record=False,
                     # physics=Physics.PYB_DW,
                     neighbourhood_radius=10,
                     pyb_freq=ARGS.simulation_freq_hz,
                     ctrl_freq=ARGS.control_freq_hz,
                     gui=ARGS.gui,
                     user_debug_gui=ARGS.user_debug_gui
                         )


    #### Obtain the PyBullet Client ID from the environment ####
#### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    # --- FINAL, CONFIRMED COLORING CODE BLOCK ---
    Predators_ids = env.getDroneIds()
    
    # SOLUTION: PyBullet assigns IDs sequentially.
    # Predators (Drones) are loaded first, getting IDs 0 to NUM_DRONES - 1.
    # Preys are loaded second, getting IDs NUM_DRONES to (NUM_DRONES + PREYS) - 1.
    
    # Calculate the sequential IDs for the Preys
    start_id = ARGS.num_drones
    end_id = ARGS.num_drones + ARGS.preys
    Preys_ids = list(range(start_id, end_id))

    # 1. Color the Predators (Red)
    red = [1.0, 0.0, 0.0, 1.0] # Bright Red
    for drone_id in Predators_ids:
        # Use objectUniqueId, as required by your PyBullet version
        p.changeVisualShape(objectUniqueId=drone_id, 
                            linkIndex=-1, 
                            rgbaColor=red, 
                            physicsClientId=PYB_CLIENT)

    # 2. Color the Preys (Blue)
    blue = [0.0, 0.0, 1.0, 1.0] # Bright Blue
    for prey_id in Preys_ids:
        # Use objectUniqueId, as required by your PyBullet version
        p.changeVisualShape(objectUniqueId=prey_id, 
                            linkIndex=-1, 
                            rgbaColor=blue, 
                            physicsClientId=PYB_CLIENT)

    ctrl       = [DSLPIDControl(drone_model=ARGS.drone) for i in range(ARGS.num_drones)]
    ctrl_preys = [DSLPIDControl(drone_model=ARGS.drone) for i in range(ARGS.preys)]


    if self_log:
        log_pos_xs         = np.zeros([NUM_DRONES, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_ys         = np.zeros([NUM_DRONES, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_zs         = np.zeros([NUM_DRONES, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_hxs        = np.zeros([NUM_DRONES, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_hys        = np.zeros([NUM_DRONES, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_hzs        = np.zeros([NUM_DRONES, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])


        log_pos_xs_preys   = np.zeros([PREYS, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_ys_preys   = np.zeros([PREYS, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_zs_preys   = np.zeros([PREYS, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_h_xc_preys = np.zeros([PREYS, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_h_yc_preys = np.zeros([PREYS, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])
        log_pos_h_zc_preys = np.zeros([PREYS, DURATION_SEC*DEFAULT_CONTROL_FREQ_HZ])



    START = time.time()
    action = np.zeros((NUM_DRONES, 4))
    action_preys = np.zeros((PREYS, 4))



    Predators_ids = env.getDroneIds()
    # print(f"Predators_ids: {Predators_ids}")
    


    pos_x = np.zeros(NUM_DRONES)
    pos_y = np.zeros(NUM_DRONES)
    pos_z = np.zeros(NUM_DRONES)


    pos_x_preys = np.zeros(PREYS)
    pos_y_preys = np.zeros(PREYS)
    pos_z_preys = np.zeros(PREYS)


    for i in range(0, int(ARGS.duration_sec * env.CTRL_FREQ)):
        obs, obs_preys, reward, reward_preys, done, done_preys, info, info_preys, _, _ = env.step(action, action_preys)
        
        for j, k in zip(range(NUM_DRONES), range(PREYS)):
            states = env._getDroneStateVector(j)
            # for k in range(PREYS):
            states_preys = env._getDroneStateVectorPreys(k)
            pos_x[j] = states[0]
            pos_y[j] = states[1]
            pos_z[j] = states[2]
            pos_x_preys[k] = states_preys[0]
            pos_y_preys[k] = states_preys[1]
            pos_z_preys[k] = states_preys[2]


        # obs_preys, reward_preys, done_preys, info_preys, _ = env.step_preys(action_preys)
        


        midpoint_x = (pos_x[0] + pos_x[1] + pos_x[2]) / 3
        midpoint_y = (pos_y[0] + pos_y[1] + pos_y[2]) / 3
        midpoint_z = (pos_z[0] + pos_z[1] + pos_z[2]) / 3


        midpoint_x_preys = (pos_x_preys[0] + pos_x_preys[1] + pos_x_preys[2]) / 3
        midpoint_y_preys = (pos_y_preys[0] + pos_y_preys[1] + pos_y_preys[2]) / 3
        midpoint_z_preys = (pos_z_preys[0] + pos_z_preys[1] + pos_z_preys[2]) / 3


        midpoint_x = (midpoint_x + midpoint_x_preys) / 2
        midpoint_y = (midpoint_y + midpoint_y_preys) / 2
        midpoint_z = (midpoint_z + midpoint_z_preys) / 2


        p.resetDebugVisualizerCamera(cameraDistance=10, 
                                        cameraYaw=0, 
                                        cameraPitch=-89.9, 
                                        cameraTargetPosition=[midpoint_x, midpoint_y, 0])


        # p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=-30, cameraPitch=-40,
        #                                  cameraTargetPosition=[pos_x[1], pos_y[1], pos_z[1] - 1])


        # p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=-30, cameraPitch=-40,
        #                               cameraTargetPosition=[pos_x_preys[0], pos_y_preys[0], pos_z_preys[0] - 1])


        f_util.calc_dij(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys)
        f_util.calc_ang_ij(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys)
        f_util.calc_grad_vals(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys, _3D)
        f_util.calc_p_forces()
        f_util.calc_p_forcesADM()
        f_util.calc_repulsion_predator_forces(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys)
        # f_util.calc_alignment_forces()


        # else:
        #     f_util.calc_boun_X_rep_preds(pos_xs, pos_ys, pos_zs)
        #     f_util.calc_boun_X_rep_preys(pos_x_preys, pos_y_preys, pos_z_preys)
        u, u_preys = f_util.calc_u_w()
        pos_hxs, pos_hys, pos_hzs, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys = f_util.get_heading()
        if not BOUNDLESS:
            # print('position Predators beforw:', pos_x, pos_y, pos_z)
            f_util.calc_boun_rep(pos_x, pos_y, pos_z, pos_hxs, pos_hys, pos_hzs, preds=True)
            f_util.calc_boun_rep(pos_x_preys, pos_y_preys, pos_z_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys, preds=False)
            
        f_util.update_heading()
        
        if PLOT:
            if i % (env.CTRL_FREQ*1) == 0:
                f_util.plot_swarm(pos_x, 
                                pos_y, 
                                pos_z, 
                                pos_hxs, 
                                pos_hys,
                                pos_hzs,
                                    pos_x_preys,
                                    pos_y_preys,
                                    pos_z_preys,
                                    pos_h_xc_preys,
                                    pos_h_yc_preys,
                                    pos_h_zc_preys)


        # 3D version !!! it works but it is more difficult to tune !!!!
        if _3D:    
            for j, k in zip(range(NUM_DRONES), range(PREYS)):
                # for k in range(PREYS):
                # print(j,k)
                vel_cmd = np.array([u[j]*np.cos(pos_hxs[j]), u[j]*np.cos(pos_hys[j]), u[j]*np.cos(pos_hzs[j])])
                pos_cmd = np.array([pos_x[j], pos_y[j], pos_z[j]])


                action[j], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                    state=obs[j],
                                                                    target_pos=pos_cmd,
                                                                    target_vel=vel_cmd,
                                                                    target_rpy=np.array([0, 0, 0])
                                                                    )
                vel_cmd_preys = np.array([u_preys[k]*np.cos(pos_h_xc_preys[k]), u_preys[k]*np.cos(pos_h_yc_preys[k]), u_preys[k]*np.cos(pos_h_zc_preys[k])])
                pos_cmd_preys = np.array([pos_x_preys[k], pos_y_preys[k], pos_z_preys[k]])


                action_preys[k], _, _ = ctrl_preys[k].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                                state=obs_preys[k],
                                                                                target_pos=pos_cmd_preys,
                                                                                target_vel=vel_cmd_preys,
                                                                                target_rpy=np.array([0, 0, 0])
                                                                                )
        else:
            for j, k in zip(range(NUM_DRONES), range(PREYS)):
                vel_cmd = np.array([u[j] * np.cos(pos_hxs[j]), u[j] * np.cos(pos_hys[j]), 0])
                pos_cmd = np.array([pos_x[j], pos_y[j], pos_z[j]])  # Keep z unchanged


                action[j], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                state=obs[j],
                                                                target_pos=pos_cmd,
                                                                target_vel=vel_cmd,
                                                                target_rpy=np.array([0, 0, 0])
                                                                )
                vel_cmd_preys = np.array([u_preys[k] * np.cos(pos_h_xc_preys[k]), u_preys[k] * np.cos(pos_h_yc_preys[k]), 0])
                pos_cmd_preys = np.array([pos_x_preys[k], pos_y_preys[k], pos_z_preys[k]])  # Keep z unchanged


                action_preys[k], _, _ = ctrl_preys[k].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                            state=obs_preys[k],
                                                                            target_pos=pos_cmd_preys,
                                                                            target_vel=vel_cmd_preys,
                                                                            target_rpy=np.array([0, 0, 0])
                                                                            ) 



        #### Log the simulation ####################################
        if self_log:
            log_pos_xs[:, i]  = pos_x[:]
            log_pos_ys[:, i]  = pos_y[:]
            log_pos_zs[:, i]  = pos_z[:]
            log_pos_hxs[:, i] = pos_hxs[:]
            log_pos_hys[:, i] = pos_hys[:]
            log_pos_hzs[:, i] = pos_hzs[:]


            log_pos_xs_preys[:, i]   = pos_x_preys[:]
            log_pos_ys_preys[:, i]   = pos_y_preys[:]
            log_pos_zs_preys[:, i]   = pos_z_preys[:]
            log_pos_h_xc_preys[:, i] = pos_h_xc_preys[:]
            log_pos_h_yc_preys[:, i] = pos_h_yc_preys[:]
            log_pos_h_zc_preys[:, i] = pos_h_zc_preys[:]



        #### Printout ##############################################
        env.render()


        #### Sync the simulation ###################################
        if gui:
            pass
            sync(i, START, env.CTRL_TIMESTEP)


    #### Close the environment #################################
    env.close()


    #### Save the simulation results ###########################
    if self_log:
        dt_string = datetime.now().strftime("%d-%m-%Y_%H-%M")
        seed_str = f"_seed-{seed}" if seed is not None else ""
        save_dir = "data/" + f'flocking3D-{_3D}_percNoSensor-{int(PERC_NO_SENSOR*100)}%{seed_str}_' + dt_string + "/"


        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        
        filename_posx = save_dir    + "log_pos_xs_"  + ".npy"
        filename_posy = save_dir    + "log_pos_ys_"  + ".npy"
        filename_posz = save_dir    + "log_pos_zs_"  + ".npy"
        filename_pos_hxs = save_dir + "log_pos_hxs_" + ".npy"
        filename_pos_hys = save_dir + "log_pos_hys_" + ".npy"
        filename_pos_hzs = save_dir + "log_pos_hzs_" + ".npy"


        filename_posx_preys = save_dir    + "log_pos_xs_preys_"  + ".npy"
        filename_posy_preys = save_dir    + "log_pos_ys_preys_"  + ".npy"
        filename_posz_preys = save_dir    + "log_pos_zs_preys_"  + ".npy"
        filename_pos_hx_preys = save_dir + "log_pos_h_xc_preys_" + ".npy"
        filename_pos_hy_preys = save_dir + "log_pos_h_yc_preys_" + ".npy"
        filename_pos_hz_preys = save_dir + "log_pos_h_zc_preys_" + ".npy"


        np.save(filename_posx, log_pos_xs.T)
        np.save(filename_posy, log_pos_ys.T)
        np.save(filename_posz, log_pos_zs.T)
        np.save(filename_pos_hxs, log_pos_hxs.T)
        np.save(filename_pos_hys, log_pos_hys.T)
        np.save(filename_pos_hzs, log_pos_hzs.T)


        np.save(filename_posx_preys, log_pos_xs_preys.T)
        np.save(filename_posy_preys, log_pos_ys_preys.T)
        np.save(filename_posz_preys, log_pos_zs_preys.T)
        np.save(filename_pos_hx_preys, log_pos_h_xc_preys.T)
        np.save(filename_pos_hy_preys, log_pos_h_yc_preys.T)
        np.save(filename_pos_hz_preys, log_pos_h_zc_preys.T)

        # Save seed information for reproducibility
        if seed is not None:
            seed_info_file = save_dir + "simulation_info.txt"
            with open(seed_info_file, 'w') as f:
                f.write(f"Random seed used: {seed}\n")
                f.write(f"Simulation date: {dt_string}\n")
                f.write(f"3D mode: {_3D}\n")
                f.write(f"Boundless mode: {BOUNDLESS}\n")
                f.write(f"Number of drones: {NUM_DRONES}\n")
                f.write(f"Number of preys: {PREYS}\n")
                f.write(f"Percentage no sensor: {PERC_NO_SENSOR}\n")




if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--drone', default=DEFAULT_DRONES, type=DroneModel, help='Drone model (default: BETA)',
                        metavar='', choices=DroneModel)
    parser.add_argument('--num_drones', default=NUM_DRONES, type=int, help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--preys', default=PREYS, type=int, help='Number of preys (default: 3)', metavar='')
    parser.add_argument('--physics', default=DEFAULT_PHYSICS, type=Physics, help='Physics updates (default: PYB)',
                        metavar='', choices=Physics)
    parser.add_argument('--gui', default=DEFAULT_GUI, type=str2bool, help='Whether to use PyBullet GUI (default: True)',
                        metavar='')
    parser.add_argument('--plot', default=DEFAULT_PLOT, type=str2bool,
                        help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui', default=DEFAULT_USER_DEBUG_GUI, type=str2bool,
                        help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=DEFAULT_SIMULATION_FREQ_HZ, type=int,
                        help='Simulation frequency in Hz (default: 500)', metavar='')
    parser.add_argument('--control_freq_hz', default=DEFAULT_CONTROL_FREQ_HZ, type=int,
                        help='Control frequency in Hz (default: 25)', metavar='')
    parser.add_argument('--output_folder', default=DEFAULT_OUTPUT_FOLDER, type=str,
                        help='Folder where to save logs (default: "results")', metavar='')
    parser.add_argument('--duration_sec',default=DURATION_SEC, type=int,
                        help='Duration of the simulation in seconds (default: 5)', metavar='')
    # Add the new seed parameter
    parser.add_argument('--seed', default=DEFAULT_SEED, type=int,
                        help='Random seed for reproducible results (default: None)', metavar='')
    ARGS = parser.parse_args()


    run(**vars(ARGS))

##json file
# """CrazyFlie software-in-the-loop control example with JSON configuration support.

# Setup
# -----
# Step 1: Clone pycffirmware from [https://github.com/utiasDSL/pycffirmware](https://github.com/utiasDSL/pycffirmware)
# Step 2: Follow the install instructions for pycffirmware in its README
# Step 3: Create config.json file or use default configuration

# Example
# -------
# In terminal, run:
# python gym_pybullet_drones/examples/cf.py --config config.json --seed 42
# python gym_pybullet_drones/examples/cf.py --config config_small_swarm.json
# """

# import os
# import sys

# current_dir = os.path.dirname(os.path.realpath(__file__))
# parent_dir = os.path.dirname(current_dir)
# sys.path.insert(0, parent_dir)

# import time
# import argparse
# import stat
# import numpy as np
# from utils.enums import DroneModel, Physics
# from envs.CrtlAviary_prey_preds import CtrlAviary_
# from control.DSLPIDControl import DSLPIDControl
# from utils.utils import sync, str2bool
# from flocking import FlockingUtils
# import pybullet as p
# from datetime import datetime
# import re

# # Import our configuration loader
# from config_loader import ConfigLoader

# # Global configuration object
# CONFIG = None

# def load_configuration(config_path="config.json"):
#     """Load configuration and set global parameters."""
#     global CONFIG
    
#     try:
#         CONFIG = ConfigLoader(config_path)
#         print(f"Configuration loaded from: {config_path}")
#     except FileNotFoundError:
#         print(f"Configuration file {config_path} not found. Creating default configuration...")
#         create_default_config(config_path)
#         CONFIG = ConfigLoader(config_path)
    
#     # Set global parameters from config
#     sim_params = CONFIG.get_simulation_params()
#     swarm_params = CONFIG.get_swarm_params()
    
#     return sim_params, swarm_params

# def create_default_config(config_path="config.json"):
#     """Create a default configuration file."""
#     default_config = {
#         "simulation": {
#             "boundless": True,
#             "3d_mode": False,
#             "gui": True,
#             "plot": True,
#             "self_log": False,
#             "duration_sec": 30,
#             "simulation_freq_hz": 240,
#             "control_freq_hz": 48,
#             "output_folder": "results"
#         },
#         "swarm": {
#             "num_drones": 10,
#             "num_preys": 10,
#             "perc_no_sensor": 0.0,
#             "if_cube": True
#         },
#         "physics": {
#             "drone_model": "cf2x",
#             "physics_engine": "pyb",
#             "neighbourhood_radius": 10
#         },
#         "initialization": {
#             "boundless_settings": {
#                 "init_center_x": 3,
#                 "init_center_y": 3,
#                 "init_center_z": 3,
#                 "spacing": 0.95,
#                 "min_distance_multiplier": 1.7,
#                 "min_distance_factor": 0.3,
#                 "min_distance_offset": 2.0
#             },
#             "lab_settings": {
#                 "init_center_x": 1,
#                 "init_center_y": 1,
#                 "init_center_z": 2,
#                 "spacing": 0.2,
#                 "min_distance": 1.3
#             },
#             "3d_settings": {
#                 "min_distance": 3.0
#             }
#         },
#         "camera": {
#             "distance": 5,
#             "yaw": -25,
#             "pitch": -20
#         },
#         "logging": {
#             "enable": False,
#             "data_folder": "data/"
#         }
#     }
    
#     import json
#     with open(config_path, 'w') as f:
#         json.dump(default_config, f, indent=2)
#     print(f"Default configuration created at: {config_path}")

# def calculate_min_distance():
#     """Calculate minimum distance based on configuration."""
#     if CONFIG.get('simulation.3d_mode'):
#         return CONFIG.get('initialization.3d_settings.min_distance', 3.0)
#     elif CONFIG.get('simulation.boundless'):
#         num_preys = CONFIG.get('swarm.num_preys', 10)
#         perc_no_sensor = CONFIG.get('swarm.perc_no_sensor', 0.0)
#         multiplier = CONFIG.get('initialization.boundless_settings.min_distance_multiplier', 1.7)
#         factor = CONFIG.get('initialization.boundless_settings.min_distance_factor', 0.3)
#         offset = CONFIG.get('initialization.boundless_settings.min_distance_offset', 2.0)
        
#         return (np.sqrt(num_preys) * multiplier * factor) + 3/offset
#     else:
#         return CONFIG.get('initialization.lab_settings.min_distance', 1.3)

# def initialize_positions_with_seed(seed=None):
#     """Initialize positions with optional random seed for reproducibility."""
#     if seed is not None:
#         np.random.seed(seed)
#         print(f"Random seed set to: {seed}")
    
#     # Get parameters from config
#     _3D = CONFIG.get('simulation.3d_mode', False)
#     BOUNDLESS = CONFIG.get('simulation.boundless', True)
#     NUM_DRONES = CONFIG.get('swarm.num_drones', 10)
#     PREYS = CONFIG.get('swarm.num_preys', 10)
#     PERC_NO_SENSOR = CONFIG.get('swarm.perc_no_sensor', 0.0)
    
#     min_distance = calculate_min_distance()
    
#     if BOUNDLESS:
#         init_params = CONFIG.get_init_params()
#         init_center_x = init_params['init_center_x']
#         init_center_y = init_params['init_center_y'] 
#         init_center_z = init_params['init_center_z']
#         spacing = init_params['spacing']
        
#         init_center_x_preys = init_center_x + np.random.choice([-1, 1]) * min_distance
#         init_center_y_preys = init_center_y + np.random.choice([-1, 1]) * min_distance
#         init_center_z_preys = init_center_z + 0
#     else:
#         init_params = CONFIG.get_init_params()
#         init_center_x = init_params['init_center_x']
#         init_center_y = init_params['init_center_y']
#         init_center_z = init_params['init_center_z']
#         spacing = init_params['spacing']
        
#         init_center_x_preys = init_center_x + min_distance
#         init_center_y_preys = init_center_y + min_distance
#         init_center_z_preys = init_center_z + 0

#     drones_ids = list(range(NUM_DRONES))
    
#     f_util = FlockingUtils(NUM_DRONES,
#                            PREYS, 
#                            init_center_x, 
#                            init_center_y, 
#                            init_center_z,
#                            spacing, 
#                            init_center_x_preys, 
#                            init_center_y_preys, 
#                            init_center_z_preys, 
#                            drones_ids=drones_ids, 
#                            perc_no_sensor=PERC_NO_SENSOR,
#                            boundless=BOUNDLESS,
#                            _3D=_3D)

#     if _3D:
#         pos_xs, pos_ys, pos_zs, pos_hxs, pos_hys, pos_hzs = f_util.initialize_positions3D(preds=True)
#         pos_xs_preys, pos_ys_preys, pos_zs_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys = f_util.initialize_positions3D(preds=False)
#     else:
#         pos_xs, pos_ys, pos_zs, pos_h_xc, pos_h_yc, pos_h_zc = f_util.initialize_positions(preds=True)
#         pos_xs_preys, pos_ys_preys, pos_zs_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys = f_util.initialize_positions(preds=False)

#     return f_util, pos_xs, pos_ys, pos_zs, pos_xs_preys, pos_ys_preys, pos_zs_preys

# def run(config_path="config.json", seed=None, **override_args):
#     """Run the simulation with configuration support."""
#     global CONFIG
    
#     # Load configuration
#     sim_params, swarm_params = load_configuration(config_path)
    
#     # Override with command line arguments if provided
#     for key, value in override_args.items():
#         if value is not None:
#             # Map argument names to config paths
#             config_mapping = {
#                 'duration_sec': 'simulation.duration_sec',
#                 'gui': 'simulation.gui',
#                 'plot': 'simulation.plot',
#                 'num_drones': 'swarm.num_drones',
#                 'preys': 'swarm.num_preys',
#                 # Add more mappings as needed
#             }
            
#             if key in config_mapping:
#                 CONFIG.set(config_mapping[key], value)
#                 print(f"Override: {config_mapping[key]} = {value}")
    
#     # Get updated parameters after overrides
#     sim_params = CONFIG.get_simulation_params()
#     swarm_params = CONFIG.get_swarm_params()
#     physics_params = CONFIG.get_physics_params()
#     camera_params = CONFIG.get_camera_params()
    
#     # Extract parameters
#     BOUNDLESS = sim_params['boundless']
#     _3D = sim_params['_3d']
#     GUI = sim_params['gui']
#     PLOT = sim_params['plot']
#     self_log = sim_params['self_log']
#     DURATION_SEC = sim_params['duration_sec']
#     SIMULATION_FREQ_HZ = sim_params['simulation_freq_hz']
#     CONTROL_FREQ_HZ = sim_params['control_freq_hz']
    
#     NUM_DRONES = swarm_params['num_drones']
#     PREYS = swarm_params['num_preys']
    
#     print(f"Running simulation with:")
#     print(f"  - Drones: {NUM_DRONES}, Preys: {PREYS}")
#     print(f"  - Duration: {DURATION_SEC}s")
#     print(f"  - 3D Mode: {_3D}, Boundless: {BOUNDLESS}")
#     print(f"  - GUI: {GUI}, Plot: {PLOT}")
#     if seed:
#         print(f"  - Seed: {seed}")
    
#     # Initialize positions with seed
#     f_util, pos_xs, pos_ys, pos_zs, pos_xs_preys, pos_ys_preys, pos_zs_preys = initialize_positions_with_seed(seed)

#     INIT_XYZ = np.zeros([NUM_DRONES, 3])
#     INIT_XYZ[:, 0] = pos_xs
#     INIT_XYZ[:, 1] = pos_ys
#     INIT_XYZ[:, 2] = pos_zs
#     INIT_RPY = np.array([[.0, .0, .0] for _ in range(NUM_DRONES)])

#     INIT_XYZ_PREYS = np.zeros([PREYS, 3])
#     INIT_XYZ_PREYS[:, 0] = pos_xs_preys
#     INIT_XYZ_PREYS[:, 1] = pos_ys_preys
#     INIT_XYZ_PREYS[:, 2] = pos_zs_preys
#     INIT_RPY_PREYS = np.array([[.0, .0, .0] for _ in range(PREYS)])

#     # Convert string parameters to enums
#     drone_model = DroneModel(physics_params['drone_model'])
#     physics_engine = Physics(physics_params['physics_engine'])

#     env = CtrlAviary_(drone_model=drone_model,
#                      num_drones=NUM_DRONES,
#                      preys=PREYS,
#                      initial_xyzs=INIT_XYZ,
#                      initial_rpys=INIT_RPY,
#                      initial_xyzs_preys=INIT_XYZ_PREYS,
#                      initial_rpys_preys=INIT_RPY_PREYS,
#                      physics=physics_engine,
#                      record=False,
#                      neighbourhood_radius=physics_params['neighbourhood_radius'],
#                      pyb_freq=SIMULATION_FREQ_HZ,
#                      ctrl_freq=CONTROL_FREQ_HZ,
#                      gui=GUI,
#                      user_debug_gui=GUI
#                      )

#     #### Obtain the PyBullet Client ID from the environment ####
#     PYB_CLIENT = env.getPyBulletClient()

#     ctrl = [DSLPIDControl(drone_model=drone_model) for i in range(NUM_DRONES)]
#     ctrl_preys = [DSLPIDControl(drone_model=drone_model) for i in range(PREYS)]

#     # Initialize logging arrays if needed
#     if self_log:
#         log_pos_xs = np.zeros([NUM_DRONES, DURATION_SEC*CONTROL_FREQ_HZ])
#         log_pos_ys = np.zeros([NUM_DRONES, DURATION_SEC*CONTROL_FREQ_HZ])
#         log_pos_zs = np.zeros([NUM_DRONES, DURATION_SEC*CONTROL_FREQ_HZ])
#         log_pos_hxs = np.zeros([NUM_DRONES, DURATION_SEC*CONTROL_FREQ_HZ])
#         log_pos_hys = np.zeros([NUM_DRONES, DURATION_SEC*CONTROL_FREQ_HZ])
#         log_pos_hzs = np.zeros([NUM_DRONES, DURATION_SEC*CONTROL_FREQ_HZ])

#         log_pos_xs_preys = np.zeros([PREYS, DURATION_SEC*CONTROL_FREQ_HZ])
#         log_pos_ys_preys = np.zeros([PREYS, DURATION_SEC*CONTROL_FREQ_HZ])
#         log_pos_zs_preys = np.zeros([PREYS, DURATION_SEC*CONTROL_FREQ_HZ])
#         log_pos_h_xc_preys = np.zeros([PREYS, DURATION_SEC*CONTROL_FREQ_HZ])
#         log_pos_h_yc_preys = np.zeros([PREYS, DURATION_SEC*CONTROL_FREQ_HZ])
#         log_pos_h_zc_preys = np.zeros([PREYS, DURATION_SEC*CONTROL_FREQ_HZ])

#     START = time.time()
#     action = np.zeros((NUM_DRONES, 4))
#     action_preys = np.zeros((PREYS, 4))

#     Predators_ids = env.getDroneIds()
    
#     pos_x = np.zeros(NUM_DRONES)
#     pos_y = np.zeros(NUM_DRONES)
#     pos_z = np.zeros(NUM_DRONES)

#     pos_x_preys = np.zeros(PREYS)
#     pos_y_preys = np.zeros(PREYS)
#     pos_z_preys = np.zeros(PREYS)

#     # Main simulation loop
#     for i in range(0, int(DURATION_SEC * env.CTRL_FREQ)):
#         obs, obs_preys, reward, reward_preys, done, done_preys, info, info_preys, _, _ = env.step(action, action_preys)
        
#         for j, k in zip(range(NUM_DRONES), range(PREYS)):
#             states = env._getDroneStateVector(j)
#             states_preys = env._getDroneStateVectorPreys(k)
#             pos_x[j] = states[0]
#             pos_y[j] = states[1]
#             pos_z[j] = states[2]
#             pos_x_preys[k] = states_preys[0]
#             pos_y_preys[k] = states_preys[1]
#             pos_z_preys[k] = states_preys[2]

#         # Camera tracking
#         midpoint_x = (pos_x[0] + pos_x[1] + pos_x[2]) / 3
#         midpoint_y = (pos_y[0] + pos_y[1] + pos_y[2]) / 3
#         midpoint_z = (pos_z[0] + pos_z[1] + pos_z[2]) / 3

#         midpoint_x_preys = (pos_x_preys[0] + pos_x_preys[1] + pos_x_preys[2]) / 3
#         midpoint_y_preys = (pos_y_preys[0] + pos_y_preys[1] + pos_y_preys[2]) / 3
#         midpoint_z_preys = (pos_z_preys[0] + pos_z_preys[1] + pos_z_preys[2]) / 3

#         midpoint_x = (midpoint_x + midpoint_x_preys) / 2
#         midpoint_y = (midpoint_y + midpoint_y_preys) / 2
#         midpoint_z = (midpoint_z + midpoint_z_preys) / 2

#         p.resetDebugVisualizerCamera(cameraDistance=camera_params['distance'], 
#                                     cameraYaw=camera_params['yaw'], 
#                                     cameraPitch=camera_params['pitch'],
#                                     cameraTargetPosition=[midpoint_x, midpoint_y, midpoint_z])

#         # Flocking calculations
#         f_util.calc_dij(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys)
#         f_util.calc_ang_ij(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys)
#         f_util.calc_grad_vals(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys, _3D)
#         f_util.calc_p_forces()
#         f_util.calc_p_forcesADM()
#         f_util.calc_repulsion_predator_forces(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys)

#         u, u_preys = f_util.calc_u_w()
#         pos_hxs, pos_hys, pos_hzs, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys = f_util.get_heading()
        
#         if not BOUNDLESS:
#             f_util.calc_boun_rep(pos_x, pos_y, pos_z, pos_hxs, pos_hys, pos_hzs, preds=True)
#             f_util.calc_boun_rep(pos_x_preys, pos_y_preys, pos_z_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys, preds=False)
            
#         f_util.update_heading()
        
#         if PLOT:
#             if i % (env.CTRL_FREQ*1) == 0:
#                 f_util.plot_swarm(pos_x, 
#                                 pos_y, 
#                                 pos_z, 
#                                 pos_hxs, 
#                                 pos_hys,
#                                 pos_hzs,
#                                 pos_x_preys,
#                                 pos_y_preys,
#                                 pos_z_preys,
#                                 pos_h_xc_preys,
#                                 pos_h_yc_preys,
#                                 pos_h_zc_preys)

#         # Control calculations
#         if _3D:    
#             for j, k in zip(range(NUM_DRONES), range(PREYS)):
#                 vel_cmd = np.array([u[j]*np.cos(pos_hxs[j]), u[j]*np.cos(pos_hys[j]), u[j]*np.cos(pos_hzs[j])])
#                 pos_cmd = np.array([pos_x[j], pos_y[j], pos_z[j]])

#                 action[j], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
#                                                                 state=obs[j],
#                                                                 target_pos=pos_cmd,
#                                                                 target_vel=vel_cmd,
#                                                                 target_rpy=np.array([0, 0, 0])
#                                                                 )
#                 vel_cmd_preys = np.array([u_preys[k]*np.cos(pos_h_xc_preys[k]), u_preys[k]*np.cos(pos_h_yc_preys[k]), u_preys[k]*np.cos(pos_h_zc_preys[k])])
#                 pos_cmd_preys = np.array([pos_x_preys[k], pos_y_preys[k], pos_z_preys[k]])

#                 action_preys[k], _, _ = ctrl_preys[k].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
#                                                                             state=obs_preys[k],
#                                                                             target_pos=pos_cmd_preys,
#                                                                             target_vel=vel_cmd_preys,
#                                                                             target_rpy=np.array([0, 0, 0])
#                                                                             )
#         else:
#             for j, k in zip(range(NUM_DRONES), range(PREYS)):
#                 vel_cmd = np.array([u[j] * np.cos(pos_hxs[j]), u[j] * np.cos(pos_hys[j]), 0])
#                 pos_cmd = np.array([pos_x[j], pos_y[j], pos_z[j]])

#                 action[j], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
#                                                                 state=obs[j],
#                                                                 target_pos=pos_cmd,
#                                                                 target_vel=vel_cmd,
#                                                                 target_rpy=np.array([0, 0, 0])
#                                                                 )
#                 vel_cmd_preys = np.array([u_preys[k] * np.cos(pos_h_xc_preys[k]), u_preys[k] * np.cos(pos_h_yc_preys[k]), 0])
#                 pos_cmd_preys = np.array([pos_x_preys[k], pos_y_preys[k], pos_z_preys[k]])

#                 action_preys[k], _, _ = ctrl_preys[k].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
#                                                                             state=obs_preys[k],
#                                                                             target_pos=pos_cmd_preys,
#                                                                             target_vel=vel_cmd_preys,
#                                                                             target_rpy=np.array([0, 0, 0])
#                                                                             ) 

#         #### Log the simulation ####################################
#         if self_log:
#             log_pos_xs[:, i] = pos_x[:]
#             log_pos_ys[:, i] = pos_y[:]
#             log_pos_zs[:, i] = pos_z[:]
#             log_pos_hxs[:, i] = pos_hxs[:]
#             log_pos_hys[:, i] = pos_hys[:]
#             log_pos_hzs[:, i] = pos_hzs[:]

#             log_pos_xs_preys[:, i] = pos_x_preys[:]
#             log_pos_ys_preys[:, i] = pos_y_preys[:]
#             log_pos_zs_preys[:, i] = pos_z_preys[:]
#             log_pos_h_xc_preys[:, i] = pos_h_xc_preys[:]
#             log_pos_h_yc_preys[:, i] = pos_h_yc_preys[:]
#             log_pos_h_zc_preys[:, i] = pos_h_zc_preys[:]

#         #### Printout ##############################################
#         env.render()

#         #### Sync the simulation ###################################
#         if GUI:
#             sync(i, START, env.CTRL_TIMESTEP)

#     #### Close the environment #################################
#     env.close()

#     #### Save the simulation results ###########################
#     if self_log:
#         dt_string = datetime.now().strftime("%d-%m-%Y_%H-%M")
#         seed_str = f"_seed-{seed}" if seed is not None else ""
#         config_name = os.path.splitext(os.path.basename(config_path))[0]
        
#         save_dir = f"data/flocking3D-{_3D}_config-{config_name}{seed_str}_{dt_string}/"

#         if not os.path.exists(save_dir):
#             os.makedirs(save_dir)
        
#         # Save position data
#         filename_posx = save_dir + "log_pos_xs_.npy"
#         filename_posy = save_dir + "log_pos_ys_.npy"
#         filename_posz = save_dir + "log_pos_zs_.npy"
#         filename_pos_hxs = save_dir + "log_pos_hxs_.npy"
#         filename_pos_hys = save_dir + "log_pos_hys_.npy"
#         filename_pos_hzs = save_dir + "log_pos_hzs_.npy"

#         filename_posx_preys = save_dir + "log_pos_xs_preys_.npy"
#         filename_posy_preys = save_dir + "log_pos_ys_preys_.npy"
#         filename_posz_preys = save_dir + "log_pos_zs_preys_.npy"
#         filename_pos_hx_preys = save_dir + "log_pos_h_xc_preys_.npy"
#         filename_pos_hy_preys = save_dir + "log_pos_h_yc_preys_.npy"
#         filename_pos_hz_preys = save_dir + "log_pos_h_zc_preys_.npy"

#         np.save(filename_posx, log_pos_xs.T)
#         np.save(filename_posy, log_pos_ys.T)
#         np.save(filename_posz, log_pos_zs.T)
#         np.save(filename_pos_hxs, log_pos_hxs.T)
#         np.save(filename_pos_hys, log_pos_hys.T)
#         np.save(filename_pos_hzs, log_pos_hzs.T)

#         np.save(filename_posx_preys, log_pos_xs_preys.T)
#         np.save(filename_posy_preys, log_pos_ys_preys.T)
#         np.save(filename_posz_preys, log_pos_zs_preys.T)
#         np.save(filename_pos_hx_preys, log_pos_h_xc_preys.T)
#         np.save(filename_pos_hy_preys, log_pos_h_yc_preys.T)
#         np.save(filename_pos_hz_preys, log_pos_h_zc_preys.T)

#         # Save configuration and simulation info
#         simulation_info = {
#             "config_file": config_path,
#             "random_seed": seed,
#             "simulation_date": dt_string,
#             "parameters": {
#                 "3d_mode": _3D,
#                 "boundless_mode": BOUNDLESS,
#                 "number_of_drones": NUM_DRONES,
#                 "number_of_preys": PREYS,
#                 "duration_sec": DURATION_SEC,
#                 "simulation_freq_hz": SIMULATION_FREQ_HZ,
#                 "control_freq_hz": CONTROL_FREQ_HZ
#             }
#         }
        
#         import json
#         with open(save_dir + "simulation_info.json", 'w') as f:
#             json.dump(simulation_info, f, indent=2)
        
#         # Save the configuration used for this run
#         CONFIG.save_config(save_dir + "config_used.json")
        
#         print(f"Results saved to: {save_dir}")

# if __name__ == "__main__":
#     #### Define and parse arguments for the script ##
#     parser = argparse.ArgumentParser(description='Drone swarm simulation with JSON configuration support')
    
#     # Configuration file argument
#     parser.add_argument('--config', default='config.json', type=str,
#                         help='Path to JSON configuration file (default: config.json)', metavar='')
    
#     # Seed argument
#     parser.add_argument('--seed', default=None, type=int,
#                         help='Random seed for reproducible results (default: None)', metavar='')
    
#     # Optional override arguments
#     parser.add_argument('--duration_sec', default=None, type=int,
#                         help='Override simulation duration in seconds', metavar='')
#     parser.add_argument('--gui', default=None, type=str2bool,
#                         help='Override GUI setting (True/False)', metavar='')
#     parser.add_argument('--plot', default=None, type=str2bool,
#                         help='Override plot setting (True/False)', metavar='')
#     parser.add_argument('--num_drones', default=None, type=int,
#                         help='Override number of predator drones', metavar='')
#     parser.add_argument('--preys', default=None, type=int,
#                         help='Override number of prey drones', metavar='')
    
#     # Utility arguments
#     parser.add_argument('--create_examples', action='store_true',
#                         help='Create example configuration files and exit')
#     parser.add_argument('--show_config', action='store_true',
#                         help='Show current configuration and exit')
    
#     ARGS = parser.parse_args()
    
#     # Handle utility commands
#     if ARGS.create_examples:
#         try:
#             from config_loader import create_default_configs
#             create_default_configs()
#             print("Example configuration files created successfully!")
#         except Exception as e:
#             print(f"Error creating example configs: {e}")
#         sys.exit(0)
    
#     if ARGS.show_config:
#         try:
#             config = ConfigLoader(ARGS.config)
#             config.print_config()
#         except Exception as e:
#             print(f"Error loading configuration: {e}")
#         sys.exit(0)
    
#     # Run simulation
#     try:
#         run(
#             config_path=ARGS.config,
#             seed=ARGS.seed,
#             duration_sec=ARGS.duration_sec,
#             gui=ARGS.gui,
#             plot=ARGS.plot,
#             num_drones=ARGS.num_drones,
#             preys=ARGS.preys
#         )
#     except KeyboardInterrupt:
#         print("\nSimulation interrupted by user.")
#     except Exception as e:
#         print(f"Error running simulation: {e}")
#         import traceback
#         traceback.print_exc()

