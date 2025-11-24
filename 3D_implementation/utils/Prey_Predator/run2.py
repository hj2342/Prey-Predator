
# """CrazyFlie software-in-the-loop control example with multi-trial analysis and JSON configuration support.

# Setup
# -----
# Step 1: Clone pycffirmware from https://github.com/utiasDSL/pycffirmware
# Step 2: Follow the install instructions for pycffirmware in its README
# Step 3: Create a config.json file with your desired parameters

# Example
# -------
# In terminal, run:
# python run2_updated.py --config config.json
# python run2_updated.py --config config.json --parameter_set aggressive_pursuit
# python run2_updated.py --config config.json --run_all_sets
# python run2_updated.py --plot_only --results_dir experiment_results/results_default_config_02-10-2025_14-30

# """

# import os
# import sys
# import json
# import argparse
# from copy import deepcopy

# current_dir = os.path.dirname(os.path.realpath(__file__))
# parent_dir = os.path.dirname(current_dir)
# sys.path.insert(0, parent_dir)

# import time
# import numpy as np
# import matplotlib.pyplot as plt
# from utils.enums import DroneModel, Physics
# from envs.CrtlAviary_prey_preds import CtrlAviary_
# from control.DSLPIDControl import DSLPIDControl
# from utils.utils import sync, str2bool
# from flocking import FlockingUtils
# import pybullet as p
# from datetime import datetime
# import pandas as pd
# from collections import defaultdict
# import pickle


# def compute_radius(positions):
#     centroid = np.mean(positions, axis=0)
#     distances = np.linalg.norm(positions - centroid, axis=1)
#     return np.max(distances)


# class ConfigManager:
#     """Manages configuration loading and parameter set merging."""
    
#     def __init__(self, config_path):
#         self.config_path = config_path
#         self.config = self.load_config()
        
#     def load_config(self):
#         """Load configuration from JSON file."""
#         try:
#             with open(self.config_path, 'r') as f:
#                 config = json.load(f)
#             print(f"Configuration loaded from: {self.config_path}")
#             return config
#         except FileNotFoundError:
#             print(f"Error: Configuration file '{self.config_path}' not found.")
#             sys.exit(1)
#         except json.JSONDecodeError as e:
#             print(f"Error: Invalid JSON in configuration file: {e}")
#             sys.exit(1)
    
#     def merge_configs(self, base_config, overrides):
#         """Recursively merge override configuration into base configuration."""
#         merged = deepcopy(base_config)
        
#         def deep_merge(base, override):
#             for key, value in override.items():
#                 if key in base and isinstance(base[key], dict) and isinstance(value, dict):
#                     deep_merge(base[key], value)
#                 else:
#                     base[key] = value
        
#         deep_merge(merged, overrides)
#         return merged
    
#     def get_parameter_set_config(self, parameter_set_name=None):
#         """Get configuration for a specific parameter set."""
#         base_config = self.config['base_config']
        
#         if parameter_set_name is None:
#             return base_config
            
#         # Find the parameter set
#         parameter_set = None
#         for pset in self.config['parameter_sets']:
#             if pset['name'] == parameter_set_name:
#                 parameter_set = pset
#                 break
                
#         if parameter_set is None:
#             available_sets = [pset['name'] for pset in self.config['parameter_sets']]
#             print(f"Error: Parameter set '{parameter_set_name}' not found.")
#             print(f"Available parameter sets: {available_sets}")
#             sys.exit(1)
        
#         # Merge base config with overrides
#         merged_config = self.merge_configs(base_config, parameter_set['overrides'])
#         merged_config['_parameter_set_name'] = parameter_set_name
#         merged_config['_parameter_set_description'] = parameter_set['description']
        
#         return merged_config
    
#     def get_all_parameter_sets(self):
#         """Get list of all available parameter sets."""
#         return [(pset['name'], pset['description']) for pset in self.config['parameter_sets']]

# class BehaviorAnalyzer:
#     def __init__(self, num_predators, num_preys, ctrl_freq, config):
#         self.num_predators = num_predators
#         self.num_preys = num_preys
#         self.ctrl_freq = ctrl_freq
#         self.follow_threshold = config['behavior_analysis']['follow_distance_threshold']
#         self.engulf_threshold = config['behavior_analysis']['engulf_distance_threshold'] 
#         self.separation_threshold = config['behavior_analysis']['separation_distance_threshold']
#         self.min_behavior_duration = config['behavior_analysis']['min_behavior_duration']
#         self.analysis_window_sec = config.get('analysis_window_sec', 150)  # Last N seconds to analyze
#         self.reset_trial()
        
#     def reset_trial(self):
#         self.distances = []
#         self.behaviors = []
#         self.current_behavior = None
#         self.behavior_start_frame = 0
#         self.behavior_counts = {
#             'following': 0,
#             'engulfing': 0,
#             'separating': 0
#         }
        
#     def update(self, pred_positions, prey_positions, frame):
#         # Compute centroids
#         pred_centroid = np.mean(pred_positions, axis=0)
#         prey_centroid = np.mean(prey_positions, axis=0)
        
#         # Distance between centroids
#         centroid_distance = np.linalg.norm(pred_centroid - prey_centroid)
        
#         self.distances.append(centroid_distance)
        
#         # Classify based on thresholds
#         new_behavior = self._classify_behavior(centroid_distance)

#         if new_behavior != self.current_behavior:
#             if (self.current_behavior is not None and 
#                 frame - self.behavior_start_frame >= self.min_behavior_duration):
#                 self.behavior_counts[self.current_behavior] += 1
#                 self.behaviors.append({
#                     'behavior': self.current_behavior,
#                     'start_frame': self.behavior_start_frame,
#                     'end_frame': frame,
#                     'duration': frame - self.behavior_start_frame
#                 })
            
#             self.current_behavior = new_behavior
#             self.behavior_start_frame = frame
            
#     def _classify_behavior(self, distance):
#         if distance <= self.engulf_threshold:
#             return 'engulfing'
#         elif distance <= self.follow_threshold:
#             return 'following'
#         elif distance >= self.separation_threshold:
#             return 'separating'
#         else:
#             return 'following'  # Default to following for intermediate distances
    
#     def finalize_trial(self, total_frames):
#         # Finalize the last behavior
#         if (self.current_behavior is not None and 
#             total_frames - self.behavior_start_frame >= self.min_behavior_duration):
#             self.behavior_counts[self.current_behavior] += 1
#             self.behaviors.append({
#                 'behavior': self.current_behavior,
#                 'start_frame': self.behavior_start_frame,
#                 'end_frame': total_frames,
#                 'duration': total_frames - self.behavior_start_frame
#             })
        
#         # Calculate behavior counts for last N seconds only
#         analysis_frames = int(self.analysis_window_sec * self.ctrl_freq)
#         start_frame = max(0, total_frames - analysis_frames)
        
#         # Re-count behaviors for the analysis window
#         windowed_behavior_counts = {
#             'following': 0,
#             'engulfing': 0,
#             'separating': 0
#         }
        
#         for behavior_event in self.behaviors:
#             # Check if behavior overlaps with analysis window
#             if behavior_event['end_frame'] >= start_frame:
#                 windowed_behavior_counts[behavior_event['behavior']] += 1
        
#         self.windowed_behavior_counts = windowed_behavior_counts
    
#     def get_results(self):
#         return {
#             'behavior_counts': self.behavior_counts.copy(),
#             'windowed_behavior_counts': self.windowed_behavior_counts.copy(),
#             'behaviors': self.behaviors.copy(),
#             'distances': self.distances.copy(),
#             'thresholds': {
#                 'follow': self.follow_threshold,
#                 'engulf': self.engulf_threshold,
#                 'separation': self.separation_threshold
#             },
#             'analysis_window_sec': self.analysis_window_sec
#         }

# def run_single_trial(trial_num, config, num_trials):
#     """Run a single trial and return behavior analysis results."""
    
#     # Extract configuration values
#     NUM_DRONES = config['ns']
#     PREYS = config['ns']
#     _3D = config['3d']
#     BOUNDLESS = config['boundless']
#     PERC_NO_SENSOR = config['perc_no_sensor']
#     DURATION_SEC = config['duration_sec']
#     DEFAULT_SIMULATION_FREQ_HZ = config['simulation']['default_simulation_freq_hz']
#     DEFAULT_CONTROL_FREQ_HZ = config['simulation']['default_control_freq_hz']
#     gui = config['gui']
    
#     # Initialize positions for this trial
#     drones_ids = list(range(NUM_DRONES))
    
#     if _3D:
#         min_distance = config['initialization']['min_distance_3d']
#     else:
#         if BOUNDLESS:
#             multiplier = config['initialization']['min_distance_formula_multiplier']
#             base = config['initialization']['min_distance_formula_base']
#             offset = config['initialization']['min_distance_formula_offset']
#             min_distance = (np.sqrt(PREYS) * multiplier * base) + offset
#         else:
#             min_distance = 1.3

#     if BOUNDLESS:
#         offset_range = config['initialization']['boundless_settings']['init_center_offset_range']
#         spacing = config['initialization']['boundless_settings']['spacing']
        
#         init_center_x = 3 + np.random.uniform(offset_range[0], offset_range[1])
#         init_center_y = 3 + np.random.uniform(offset_range[0], offset_range[1])
#         init_center_z = 3
#         init_center_x_preys = init_center_x + np.random.choice([-1, 1]) * min_distance
#         init_center_y_preys = init_center_y + np.random.choice([-1, 1]) * min_distance
#         init_center_z_preys = init_center_z + 0
#     else:
#         bounded_settings = config['initialization']['bounded_settings']
#         init_center_x = bounded_settings['init_center_x']
#         init_center_y = bounded_settings['init_center_y']
#         init_center_z = bounded_settings['init_center_z']
#         init_center_x_preys = init_center_x + min_distance
#         init_center_y_preys = init_center_y + min_distance
#         init_center_z_preys = init_center_z + 0
#         spacing = bounded_settings['spacing']

#     f_util = FlockingUtils(NUM_DRONES, PREYS, init_center_x, init_center_y, init_center_z,
#                           spacing, init_center_x_preys, init_center_y_preys, init_center_z_preys,
#                           drones_ids=drones_ids, perc_no_sensor=PERC_NO_SENSOR,
#                           boundless=BOUNDLESS, _3D=_3D)

#     # Initialize positions
#     if _3D:
#         pos_xs, pos_ys, pos_zs, pos_hxs, pos_hys, pos_hzs = f_util.initialize_positions3D(preds=True)
#         pos_xs_preys, pos_ys_preys, pos_zs_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys = f_util.initialize_positions3D(preds=False)
#     else:
#         pos_xs, pos_ys, pos_zs, pos_h_xc, pos_h_yc, pos_h_zc = f_util.initialize_positions(preds=True)
#         pos_xs_preys, pos_ys_preys, pos_zs_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys = f_util.initialize_positions(preds=False)

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

#     # Create environment
#     env = CtrlAviary_(drone_model=DroneModel("cf2x"),
#                      num_drones=NUM_DRONES,
#                      preys=PREYS,
#                      initial_xyzs=INIT_XYZ,
#                      initial_rpys=INIT_RPY,
#                      initial_xyzs_preys=INIT_XYZ_PREYS,
#                      initial_rpys_preys=INIT_RPY_PREYS,
#                      physics=Physics("pyb"),
#                      record=False,
#                      neighbourhood_radius=10,
#                      pyb_freq=DEFAULT_SIMULATION_FREQ_HZ,
#                      ctrl_freq=DEFAULT_CONTROL_FREQ_HZ,
#                      gui=gui,
#                      user_debug_gui=gui)    
#     # --- START COLORING BLOCK ---
#     if gui: # Only color if the GUI is active
#         PYB_CLIENT = env.getPyBulletClient()

#         # Predators/Drones IDs are from the environment
#         Predators_ids = env.getDroneIds()
        
#         # Preys IDs are assigned sequentially after the drones
#         start_id = NUM_DRONES
#         end_id = NUM_DRONES + PREYS
#         Preys_ids = list(range(start_id, end_id))

#         # 1. Color the Predators (Red)
#         red = [1.0, 0.0, 0.0, 1.0] 
#         for drone_id in Predators_ids:
#             p.changeVisualShape(objectUniqueId=drone_id, 
#                                 linkIndex=-1, 
#                                 rgbaColor=red, 
#                                 physicsClientId=PYB_CLIENT)

#         # 2. Color the Preys (Blue)
#         blue = [0.0, 0.0, 1.0, 1.0] 
#         for prey_id in Preys_ids:
#             p.changeVisualShape(objectUniqueId=prey_id, 
#                                 linkIndex=-1, 
#                                 rgbaColor=blue, 
#                                 physicsClientId=PYB_CLIENT)
#     # --- END COLORING BLOCK ---
    
    

#     # Controllers
#     ctrl = [DSLPIDControl(drone_model=DroneModel("cf2x")) for i in range(NUM_DRONES)]
#     ctrl_preys = [DSLPIDControl(drone_model=DroneModel("cf2x")) for i in range(PREYS)]

#     # Initialize behavior analyzer
#     analyzer = BehaviorAnalyzer(NUM_DRONES, PREYS, DEFAULT_CONTROL_FREQ_HZ, config)

#     action = np.zeros((NUM_DRONES, 4))
#     action_preys = np.zeros((PREYS, 4))

#     pos_x = np.zeros(NUM_DRONES)
#     pos_y = np.zeros(NUM_DRONES)
#     pos_z = np.zeros(NUM_DRONES)
#     pos_x_preys = np.zeros(PREYS)
#     pos_y_preys = np.zeros(PREYS)
#     pos_z_preys = np.zeros(PREYS)

#     total_frames = int(DURATION_SEC * env.CTRL_FREQ)
    
#     parameter_set_name = config.get('_parameter_set_name', 'default')
#     print(f"Running trial {trial_num + 1}/{num_trials} with parameter set '{parameter_set_name}'...")

#     for i in range(total_frames):
#         obs, obs_preys, reward, reward_preys, done, done_preys, info, info_preys, _, _ = env.step(action, action_preys)
        
#         # Get positions
#         for j, k in zip(range(NUM_DRONES), range(PREYS)):
#             states = env._getDroneStateVector(j)
#             states_preys = env._getDroneStateVectorPreys(k)
#             pos_x[j] = states[0]
#             pos_y[j] = states[1]
#             pos_z[j] = states[2]
#             pos_x_preys[k] = states_preys[0]
#             pos_y_preys[k] = states_preys[1]
#             pos_z_preys[k] = states_preys[2]

#         # Analyze behavior
#         pred_positions = np.column_stack([pos_x, pos_y, pos_z])
#         prey_positions = np.column_stack([pos_x_preys, pos_y_preys, pos_z_preys])
#         analyzer.update(pred_positions, prey_positions, i)

#         # Calculate forces and update positions
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

#         # Control actions
#         if _3D:
#             for j, k in zip(range(NUM_DRONES), range(PREYS)):
#                 vel_cmd = np.array([u[j]*np.cos(pos_hxs[j]), u[j]*np.cos(pos_hys[j]), u[j]*np.cos(pos_hzs[j])])
#                 pos_cmd = np.array([pos_x[j], pos_y[j], pos_z[j]])
#                 action[j], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
#                                                                 state=obs[j], target_pos=pos_cmd,
#                                                                 target_vel=vel_cmd, target_rpy=np.array([0, 0, 0]))
                
#                 vel_cmd_preys = np.array([u_preys[k]*np.cos(pos_h_xc_preys[k]), u_preys[k]*np.cos(pos_h_yc_preys[k]), u_preys[k]*np.cos(pos_h_zc_preys[k])])
#                 pos_cmd_preys = np.array([pos_x_preys[k], pos_y_preys[k], pos_z_preys[k]])
#                 action_preys[k], _, _ = ctrl_preys[k].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
#                                                                             state=obs_preys[k], target_pos=pos_cmd_preys,
#                                                                             target_vel=vel_cmd_preys, target_rpy=np.array([0, 0, 0]))
#         else:
#             for j, k in zip(range(NUM_DRONES), range(PREYS)):
#                 vel_cmd = np.array([u[j] * np.cos(pos_hxs[j]), u[j] * np.cos(pos_hys[j]), 0])
#                 pos_cmd = np.array([pos_x[j], pos_y[j], pos_z[j]])
#                 action[j], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
#                                                                 state=obs[j], target_pos=pos_cmd,
#                                                                 target_vel=vel_cmd, target_rpy=np.array([0, 0, 0]))
                
#                 vel_cmd_preys = np.array([u_preys[k] * np.cos(pos_h_xc_preys[k]), u_preys[k] * np.cos(pos_h_yc_preys[k]), 0])
#                 pos_cmd_preys = np.array([pos_x_preys[k], pos_y_preys[k], pos_z_preys[k]])
#                 action_preys[k], _, _ = ctrl_preys[k].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
#                                                                             state=obs_preys[k], target_pos=pos_cmd_preys,
#                                                                             target_vel=vel_cmd_preys, target_rpy=np.array([0, 0, 0]))

#         if gui:
#             # Calculate the current midpoint (camera target)
#             midpoint_x = (np.sum(pos_x) + np.sum(pos_x_preys)) / (NUM_DRONES + PREYS)
#             midpoint_y = (np.sum(pos_y) + np.sum(pos_y_preys)) / (NUM_DRONES + PREYS)
            
#             # Optimized Top-Down View (Zoomed)
#             p.resetDebugVisualizerCamera(cameraDistance=7.5, # Adjust for desired zoom (e.g., 10-40)
#                                          cameraYaw=0, 
#                                          cameraPitch=-89.9, # Straight down
#                                          cameraTargetPosition=[midpoint_x, midpoint_y, 0])
#             env.render()

#     # Finalize analysis
#     analyzer.finalize_trial(total_frames)
    
#     # Close environment
#     env.close()
    
#     return analyzer.get_results()

# def run_multiple_trials(config):
#     """Run multiple trials and collect results."""
#     all_results = []
#     num_trials = config['num_trials']
    
#     start_time = time.time()
    
#     for trial in range(num_trials):
#         trial_results = run_single_trial(trial, config, num_trials)
#         trial_results['trial'] = trial
#         all_results.append(trial_results)
    
#     end_time = time.time()
#     parameter_set_name = config.get('_parameter_set_name', 'default')
#     print(f"\nCompleted {num_trials} trials for parameter set '{parameter_set_name}' in {end_time - start_time:.2f} seconds")
    
#     return all_results

# def analyze_results(all_results, config):
#     """Analyze and summarize results across all trials - using windowed data."""
    
#     # Collect windowed behavior counts (last N seconds only)
#     following_counts = [r['windowed_behavior_counts']['following'] for r in all_results]
#     engulfing_counts = [r['windowed_behavior_counts']['engulfing'] for r in all_results]
#     separating_counts = [r['windowed_behavior_counts']['separating'] for r in all_results]
    
#     # Calculate statistics
#     stats = {
#         'following': {
#             'mean': np.mean(following_counts),
#             'std': np.std(following_counts),
#             'min': np.min(following_counts),
#             'max': np.max(following_counts),
#             'total': np.sum(following_counts)
#         },
#         'engulfing': {
#             'mean': np.mean(engulfing_counts),
#             'std': np.std(engulfing_counts),
#             'min': np.min(engulfing_counts),
#             'max': np.max(engulfing_counts),
#             'total': np.sum(engulfing_counts)
#         },
#         'separating': {
#             'mean': np.mean(separating_counts),
#             'std': np.std(separating_counts),
#             'min': np.min(separating_counts),
#             'max': np.max(separating_counts),
#             'total': np.sum(separating_counts)
#         }
#     }
    
#     # Print summary
#     parameter_set_name = config.get('_parameter_set_name', 'default')
#     parameter_set_desc = config.get('_parameter_set_description', 'Default configuration')
#     analysis_window = config.get('analysis_window_sec', 150)
    
#     print("\n" + "="*70)
#     print(f"BEHAVIOR ANALYSIS SUMMARY - {parameter_set_name.upper()}")
#     print("="*70)
#     print(f"Description: {parameter_set_desc}")
#     print(f"Number of trials: {len(all_results)}")
#     print(f"Duration per trial: {config['duration_sec']} seconds")
#     print(f"Analysis window: Last {analysis_window} seconds")
#     print(f"Number of predators/preys: {config['ns']}")
#     print(f"Environment: {'3D' if config['3d'] else '2D'}, {'Boundless' if config['boundless'] else 'Bounded'}")
#     print(f"Thresholds - Follow: {config['behavior_analysis']['follow_distance_threshold']}m, "
#           f"Engulf: {config['behavior_analysis']['engulf_distance_threshold']}m, "
#           f"Separation: {config['behavior_analysis']['separation_distance_threshold']}m")
#     print(f"Flocking Parameters - Alpha: {config['flocking_parameters']['alpha']}, "
#           f"Epsilon: {config['flocking_parameters']['epsilon']}, "
#           f"Sensing Range: {config['flocking_parameters']['sensing_range']}m")
#     print()
    
#     for behavior, data in stats.items():
#         print(f"{behavior.upper()} (Last {analysis_window}s):")
#         print(f"  Total occurrences: {data['total']}")
#         print(f"  Mean per trial: {data['mean']:.2f} ± {data['std']:.2f}")
#         print(f"  Range: {data['min']} - {data['max']}")
#         print()
    
#     return stats, all_results

# def create_simplified_visualizations(stats, all_results, config, save_dir):
#     """Create only the essential 3 plots: bar chart, individual trials, and mean ± std."""
    
#     parameter_set_name = config.get('_parameter_set_name', 'default')
#     control_freq = config['simulation']['default_control_freq_hz']
#     thresholds = all_results[0]['thresholds']
#     analysis_window = config.get('analysis_window_sec', 150)
    
#     # Create figure with 1 row and 3 columns
#     fig, axes = plt.subplots(1, 3, figsize=(20, 6))
#     fig.suptitle(f'Predator-Prey Behavior Analysis - {parameter_set_name} (Last {analysis_window}s)', 
#                  fontsize=16, fontweight='bold')
    
#     # 1. Bar chart of mean behavior counts (WINDOWED DATA)
#     behaviors = list(stats.keys())
#     means = [stats[b]['mean'] for b in behaviors]
#     stds = [stats[b]['std'] for b in behaviors]
    
#     ax1 = axes[0]
#     bars = ax1.bar(behaviors, means, yerr=stds, capsize=5, color=['skyblue', 'lightcoral', 'lightgreen'])
#     ax1.set_title(f'Mean Behavior Counts per Trial\n(Last {analysis_window}s)')
#     ax1.set_ylabel('Count')
#     ax1.grid(axis='y', alpha=0.3)
    
#     # Add value labels on bars
#     for i, (bar, mean, std) in enumerate(zip(bars, means, stds)):
#         ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + std + 0.1,
#                 f'{mean:.1f}', ha='center', va='bottom')
    
#     # 2. Distance Evolution for ALL Trials - Individual Lines (FULL DURATION)
#     ax2 = axes[1]
#     if all_results:
#         colors = plt.cm.tab10(np.linspace(0, 1, len(all_results)))
        
#         for i, result in enumerate(all_results):
#             distances = result['distances']
#             time_steps = np.arange(len(distances)) / control_freq
#             ax2.plot(time_steps, distances, alpha=0.6, linewidth=1, 
#                     color=colors[i], label=f'Trial {i+1}')
        
#         # Add threshold lines
#         ax2.axhline(y=thresholds['follow'], color='orange', linestyle='--', linewidth=2,
#                    label=f'Follow ({thresholds["follow"]}m)')
#         ax2.axhline(y=thresholds['engulf'], color='red', linestyle='--', linewidth=2,
#                    label=f'Engulf ({thresholds["engulf"]}m)')
#         ax2.axhline(y=thresholds['separation'], color='green', linestyle='--', linewidth=2,
#                    label=f'Separation ({thresholds["separation"]}m)')
        
#         # Highlight analysis window
#         total_time = len(all_results[0]['distances']) / control_freq
#         analysis_start_time = total_time - analysis_window
#         ax2.axvspan(analysis_start_time, total_time, alpha=0.1, color='gray', 
#                    label=f'Analysis Window ({analysis_window}s)')
        
#         ax2.set_title('Distance Evolution - All Trials')
#         ax2.set_xlabel('Time (s)')
#         ax2.set_ylabel('Min Distance (m)')
#         ax2.grid(alpha=0.3)
        
#         # Show threshold and analysis window in legend
#         handles, labels = ax2.get_legend_handles_labels()
#         important_handles = handles[-4:]
#         important_labels = labels[-4:]
#         ax2.legend(important_handles, important_labels, loc='upper right', fontsize=8)
    
#     # 3. Distance Evolution - Statistical Summary (Mean ± Std) (FULL DURATION)
#     ax3 = axes[2]
#     if all_results:
#         min_length = min(len(result['distances']) for result in all_results)
#         aligned_distances = np.array([result['distances'][:min_length] for result in all_results])
        
#         mean_distances = np.mean(aligned_distances, axis=0)
#         std_distances = np.std(aligned_distances, axis=0)
#         time_steps = np.arange(min_length) / control_freq
        
#         ax3.fill_between(time_steps, 
#                         mean_distances - std_distances,
#                         mean_distances + std_distances,
#                         alpha=0.3, color='blue', label='±1 Std Dev')
#         ax3.plot(time_steps, mean_distances, 'b-', linewidth=2, label='Mean Distance')
        
#         ax3.axhline(y=thresholds['follow'], color='orange', linestyle='--', linewidth=2,
#                    label=f'Follow ({thresholds["follow"]}m)')
#         ax3.axhline(y=thresholds['engulf'], color='red', linestyle='--', linewidth=2,
#                    label=f'Engulf ({thresholds["engulf"]}m)')
#         ax3.axhline(y=thresholds['separation'], color='green', linestyle='--', linewidth=2,
#                    label=f'Separation ({thresholds["separation"]}m)')
        
#         # Highlight analysis window
#         total_time = min_length / control_freq
#         analysis_start_time = total_time - analysis_window
#         ax3.axvspan(analysis_start_time, total_time, alpha=0.1, color='gray',
#                    label=f'Analysis Window ({analysis_window}s)')
        
#         ax3.set_title('Distance Evolution - Mean ± Std')
#         ax3.set_xlabel('Time (s)')
#         ax3.set_ylabel('Min Distance (m)')
#         ax3.legend(fontsize=8)
#         ax3.grid(alpha=0.3)
    
#     # Save the plot
#     plt.tight_layout()
#     filename = f'behavior_analysis_{parameter_set_name}.png'
#     plt.savefig(os.path.join(save_dir, filename), dpi=300, bbox_inches='tight')
#     plt.close()
#     print(f"Saved: {filename}")

# def save_results(stats, all_results, config, save_dir):
#     """Save analysis results to files including raw data for re-plotting."""
    
#     parameter_set_name = config.get('_parameter_set_name', 'default')
    
#     # Helper function to convert numpy types to Python types
#     def convert_to_python_types(obj):
#         if isinstance(obj, np.integer):
#             return int(obj)
#         elif isinstance(obj, np.floating):
#             return float(obj)
#         elif isinstance(obj, np.ndarray):
#             return obj.tolist()
#         elif isinstance(obj, dict):
#             return {key: convert_to_python_types(value) for key, value in obj.items()}
#         elif isinstance(obj, list):
#             return [convert_to_python_types(item) for item in obj]
#         else:
#             return obj
    
#     # Convert all data to Python types
#     stats_python = convert_to_python_types(stats)
#     all_results_python = convert_to_python_types(all_results)
    
#     # Save configuration used for this experiment
#     config_file = os.path.join(save_dir, f'config_{parameter_set_name}.json')
#     with open(config_file, 'w') as f:
#         config_to_save = deepcopy(config)
#         config_to_save.pop('_parameter_set_name', None)
#         config_to_save.pop('_parameter_set_description', None)
#         json.dump(config_to_save, f, indent=2)
    
#     # Save statistics as JSON (WINDOWED DATA)
#     stats_file = os.path.join(save_dir, f'behavior_statistics_{parameter_set_name}.json')
#     with open(stats_file, 'w') as f:
#         json.dump(stats_python, f, indent=2)
    
#     # Save detailed results as JSON (includes full and windowed data)
#     results_file = os.path.join(save_dir, f'detailed_results_{parameter_set_name}.json')
#     with open(results_file, 'w') as f:
#         json.dump(all_results_python, f, indent=2)
    
#     # Save raw data for plotting (PICKLE format for easy loading)
#     plot_data = {
#         'stats': stats,
#         'all_results': all_results,
#         'config': config,
#         'parameter_set_name': parameter_set_name
#     }
#     plot_data_file = os.path.join(save_dir, f'plot_data_{parameter_set_name}.pkl')
#     with open(plot_data_file, 'wb') as f:
#         pickle.dump(plot_data, f)
    
#     # Save CLEAN CSV data for easy sharing/visualization
#     # This CSV can be sent to anyone for plotting
#     csv_data_file = os.path.join(save_dir, f'distance_data_{parameter_set_name}.csv')
#     distance_data = []
    
#     for trial_idx, result in enumerate(all_results):
#         distances = result['distances']
#         control_freq = config['simulation']['default_control_freq_hz']
        
#         for frame_idx, distance in enumerate(distances):
#             distance_data.append({
#                 'trial': trial_idx + 1,
#                 'frame': frame_idx,
#                 'time_sec': frame_idx / control_freq,
#                 'distance': float(distance)
#             })
    
#     distance_df = pd.DataFrame(distance_data)
#     distance_df.to_csv(csv_data_file, index=False)
    
#     # Save summary as CSV (WINDOWED DATA)
#     summary_data = []
#     for i, result in enumerate(all_results):
#         row = {
#             'parameter_set': parameter_set_name,
#             'trial': i + 1,
#             'following_count_windowed': int(result['windowed_behavior_counts']['following']),
#             'engulfing_count_windowed': int(result['windowed_behavior_counts']['engulfing']),
#             'separating_count_windowed': int(result['windowed_behavior_counts']['separating']),
#             'following_count_full': int(result['behavior_counts']['following']),
#             'engulfing_count_full': int(result['behavior_counts']['engulfing']),
#             'separating_count_full': int(result['behavior_counts']['separating']),
#             'total_behaviors_windowed': int(sum(result['windowed_behavior_counts'].values())),
#             'total_behaviors_full': int(sum(result['behavior_counts'].values())),
#             'avg_distance': float(np.mean(result['distances'])),
#             'min_distance': float(np.min(result['distances'])),
#             'max_distance': float(np.max(result['distances'])),
#             'std_distance': float(np.std(result['distances'])),
#             'follow_threshold': float(result['thresholds']['follow']),
#             'engulf_threshold': float(result['thresholds']['engulf']),
#             'separation_threshold': float(result['thresholds']['separation']),
#             'analysis_window_sec': int(result['analysis_window_sec'])
#         }
#         summary_data.append(row)
    
#     summary_df = pd.DataFrame(summary_data)
#     summary_file = os.path.join(save_dir, f'trial_summary_{parameter_set_name}.csv')
#     summary_df.to_csv(summary_file, index=False)
#     # Save simplified TXT file with only trial, time_sec, distance (1 Hz sampling)
#     txt_file = os.path.join(save_dir, f'distance_data_{parameter_set_name}.txt')
#     with open(txt_file, 'w') as f:
#         f.write("trial\ttime_sec\tdistance\n")
#         for trial_idx, result in enumerate(all_results):
#             distances = result['distances']
#             control_freq = config['simulation']['default_control_freq_hz']
#             total_time = len(distances) // control_freq
            
#             for t in range(total_time + 1):  # include 0...duration
#                 frame_idx = t * control_freq
#                 if frame_idx < len(distances):
#                     f.write(f"{trial_idx+1}\t{t}\t{distances[frame_idx]:.6f}\n")

#     # Save metadata JSON for easy parameter reference
#     metadata = {
#         'parameter_set_name': parameter_set_name,
#         'description': config.get('_parameter_set_description', ''),
#         'num_trials': int(config['num_trials']),
#         'duration_sec': int(config['duration_sec']),
#         'analysis_window_sec': int(config.get('analysis_window_sec', 150)),
#         'swarm_size': int(config['ns']),
#         'control_freq_hz': int(config['simulation']['default_control_freq_hz']),
#         'flocking_parameters': convert_to_python_types(config['flocking_parameters']),
#         'behavior_thresholds': convert_to_python_types(config['behavior_analysis'])
#     }
    
#     metadata_file = os.path.join(save_dir, f'metadata_{parameter_set_name}.json')
#     with open(metadata_file, 'w') as f:
#         json.dump(metadata, f, indent=2)
    
#     print(f"\nResults saved to:")
#     print(f"  - Configuration: {config_file}")
#     print(f"  - Statistics (windowed): {stats_file}")
#     print(f"  - Detailed results: {results_file}")
#     print(f"  - Plot data (for re-plotting): {plot_data_file}")
#     print(f"  - Distance data CSV (shareable): {csv_data_file}")
#     print(f"  - Summary CSV: {summary_file}")
#     print(f"  - Metadata JSON: {metadata_file}")

# def load_and_plot(results_dir):
#     """Load saved plot data and regenerate plots without running simulation."""
    
#     # Find the plot data file
#     plot_data_files = [f for f in os.listdir(results_dir) if f.startswith('plot_data_') and f.endswith('.pkl')]
    
#     if not plot_data_files:
#         print(f"Error: No plot data files found in {results_dir}")
#         return
    
#     # Load the first plot data file found
#     plot_data_file = os.path.join(results_dir, plot_data_files[0])
#     print(f"Loading plot data from: {plot_data_file}")
    
#     with open(plot_data_file, 'rb') as f:
#         plot_data = pickle.load(f)
    
#     stats = plot_data['stats']
#     all_results = plot_data['all_results']
#     config = plot_data['config']
    
#     print(f"Loaded data for parameter set: {config.get('_parameter_set_name', 'unknown')}")
#     print(f"Number of trials: {len(all_results)}")
    
#     # Regenerate plots
#     create_simplified_visualizations(stats, all_results, config, results_dir)
    
#     print(f"\nPlots regenerated and saved to: {results_dir}")

# def run_experiment_with_parameter_set(config_manager, parameter_set_name, base_save_dir):
#     """Run a complete experiment with a specific parameter set."""
    
#     # Get configuration for this parameter set
#     config = config_manager.get_parameter_set_config(parameter_set_name)
    
#     parameter_set_name = config.get('_parameter_set_name', 'default')
#     print(f"\n{'='*80}")
#     print(f"RUNNING EXPERIMENT: {parameter_set_name.upper()}")
#     print(f"Description: {config.get('_parameter_set_description', 'N/A')}")
#     print(f"{'='*80}")
    
#     # Run trials
#     all_results = run_multiple_trials(config)
    
#     # Analyze results
#     stats, all_results = analyze_results(all_results, config)
    
#     # Create timestamped save directory
#     dt_string = datetime.now().strftime("%d-%m-%Y_%H-%M")
#     save_dir = os.path.join(base_save_dir, f"results_{parameter_set_name}_{dt_string}")
#     if not os.path.exists(save_dir):
#         os.makedirs(save_dir)
    
#     # Create simplified visualizations (only 3 plots)
#     create_simplified_visualizations(stats, all_results, config, save_dir)
    
#     # Save results
#     save_results(stats, all_results, config, save_dir)
    
#     print(f"\nExperiment '{parameter_set_name}' completed!")
#     print(f"Results saved to: {save_dir}")
#     print("Generated files:")
#     print(f"  - behavior_analysis_{parameter_set_name}.png (3 essential plots)")
#     print(f"  - config_{parameter_set_name}.json")
#     print(f"  - behavior_statistics_{parameter_set_name}.json (windowed data)")
#     print(f"  - detailed_results_{parameter_set_name}.json")
#     print(f"  - plot_data_{parameter_set_name}.pkl (for re-plotting)")
#     print(f"  - trial_summary_{parameter_set_name}.csv")
    
#     return {
#         'parameter_set_name': parameter_set_name,
#         'config': config,
#         'stats': stats,
#         'all_results': all_results,
#         'save_dir': save_dir
#     }

# def main():
#     """Main function to run experiments with different parameter sets."""
    
#     parser = argparse.ArgumentParser(description='Run predator-prey behavior analysis with configurable parameters')
#     parser.add_argument('--config', type=str, default='config.json',
#                        help='Path to configuration JSON file (default: config.json)')
#     parser.add_argument('--parameter_set', type=str, default=None,
#                        help='Name of specific parameter set to run (default: run default configuration)')
#     parser.add_argument('--run_all_sets', action='store_true',
#                        help='Run experiments with all parameter sets defined in config')
#     parser.add_argument('--list_sets', action='store_true',
#                        help='List available parameter sets and exit')
#     parser.add_argument('--output_dir', type=str, default='experiment_results',
#                        help='Base output directory for results (default: experiment_results)')
#     parser.add_argument('--plot_only', action='store_true',
#                        help='Only regenerate plots from saved data (no simulation)')
#     parser.add_argument('--results_dir', type=str, default=None,
#                        help='Directory containing saved plot data for --plot_only mode')
    
#     args = parser.parse_args()
    
#     # Plot-only mode
#     if args.plot_only:
#         if not args.results_dir:
#             print("Error: --results_dir required for --plot_only mode")
#             print("Example: python run2_updated.py --plot_only --results_dir experiment_results/results_default_config_02-10-2025_14-30")
#             sys.exit(1)
        
#         if not os.path.exists(args.results_dir):
#             print(f"Error: Results directory not found: {args.results_dir}")
#             sys.exit(1)
        
#         load_and_plot(args.results_dir)
#         return
    
#     # Initialize configuration manager
#     config_manager = ConfigManager(args.config)
    
#     # List parameter sets if requested
#     if args.list_sets:
#         parameter_sets = config_manager.get_all_parameter_sets()
#         print("\nAvailable parameter sets:")
#         print("-" * 50)
#         for name, description in parameter_sets:
#             print(f"  {name:<25} - {description}")
#         print()
#         return
    
#     # Create base output directory
#     base_save_dir = args.output_dir
#     if not os.path.exists(base_save_dir):
#         os.makedirs(base_save_dir)
    
#     print("Predator-Prey Multi-Trial Analysis with Configurable Parameters")
#     print(f"Configuration file: {args.config}")
#     print(f"Output directory: {base_save_dir}")
    
#     all_experiment_results = []
    
#     if args.run_all_sets:
#         # Run experiments with all parameter sets
#         parameter_sets = config_manager.get_all_parameter_sets()
#         print(f"\nRunning experiments with {len(parameter_sets)} parameter sets...")
        
#         for parameter_set_name, _ in parameter_sets:
#             experiment_result = run_experiment_with_parameter_set(
#                 config_manager, parameter_set_name, base_save_dir)
#             all_experiment_results.append(experiment_result)
    
#     else:
#         # Run experiment with single parameter set
#         parameter_set_name = args.parameter_set
#         experiment_result = run_experiment_with_parameter_set(
#             config_manager, parameter_set_name, base_save_dir)
#         all_experiment_results.append(experiment_result)
    
#     # Final summary
#     print(f"\n{'='*80}")
#     print("EXPERIMENT SUMMARY")
#     print(f"{'='*80}")
#     print(f"Total experiments completed: {len(all_experiment_results)}")
#     print(f"Results saved to: {base_save_dir}")
    
#     print("\nTo regenerate plots from saved data (without re-running simulation):")
#     for exp_result in all_experiment_results:
#         print(f"  python {sys.argv[0]} --plot_only --results_dir {exp_result['save_dir']}")
    
#     print("\nTo run specific experiments:")
#     print(f"  python {sys.argv[0]} --config {args.config} --list_sets")
#     print(f"  python {sys.argv[0]} --config {args.config} --parameter_set <name>")
#     print(f"  python {sys.argv[0]} --config {args.config} --run_all_sets")

# if __name__ == "__main__":
#     main()
"""CrazyFlie software-in-the-loop control with unified symmetric flocking parameters.

Setup
-----
Step 1: Clone pycffirmware from https://github.com/utiasDSL/pycffirmware
Step 2: Follow the install instructions for pycffirmware in its README
Step 3: Create a config.json file with your desired parameters

Example
-------
In terminal, run:
python run2.py --config config.json
python run2.py --config config.json --parameter_set aggressive_pursuit
python run2.py --config config.json --run_all_sets
python run2.py --plot_only --results_dir experiment_results/results_default_config_02-10-2025_14-30

"""

import os
import sys
import json
import argparse
from copy import deepcopy

current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

import time
import numpy as np
import matplotlib.pyplot as plt
from utils.enums import DroneModel, Physics
from envs.CrtlAviary_prey_preds import CtrlAviary_
from control.DSLPIDControl import DSLPIDControl
from utils.utils import sync, str2bool
from flocking import FlockingUtils
import pybullet as p
from datetime import datetime
import pandas as pd
from collections import defaultdict
import pickle


def compute_radius(positions):
    centroid = np.mean(positions, axis=0)
    distances = np.linalg.norm(positions - centroid, axis=1)
    return np.max(distances)


class ConfigManager:
    """Manages configuration loading and parameter set merging."""
    
    def __init__(self, config_path):
        self.config_path = config_path
        self.config = self.load_config()
        
    def load_config(self):
        """Load configuration from JSON file."""
        try:
            with open(self.config_path, 'r') as f:
                config = json.load(f)
            print(f"Configuration loaded from: {self.config_path}")
            return config
        except FileNotFoundError:
            print(f"Error: Configuration file '{self.config_path}' not found.")
            sys.exit(1)
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON in configuration file: {e}")
            sys.exit(1)
    
    def merge_configs(self, base_config, overrides):
        """Recursively merge override configuration into base configuration."""
        merged = deepcopy(base_config)
        
        def deep_merge(base, override):
            for key, value in override.items():
                if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                    deep_merge(base[key], value)
                else:
                    base[key] = value
        
        deep_merge(merged, overrides)
        return merged
    
    def get_parameter_set_config(self, parameter_set_name=None):
        """Get configuration for a specific parameter set."""
        base_config = self.config['base_config']
        
        if parameter_set_name is None:
            return base_config
            
        # Find the parameter set
        parameter_set = None
        for pset in self.config['parameter_sets']:
            if pset['name'] == parameter_set_name:
                parameter_set = pset
                break
                
        if parameter_set is None:
            available_sets = [pset['name'] for pset in self.config['parameter_sets']]
            print(f"Error: Parameter set '{parameter_set_name}' not found.")
            print(f"Available parameter sets: {available_sets}")
            sys.exit(1)
        
        # Merge base config with overrides
        merged_config = self.merge_configs(base_config, parameter_set['overrides'])
        merged_config['_parameter_set_name'] = parameter_set_name
        merged_config['_parameter_set_description'] = parameter_set['description']
        
        return merged_config
    
    def get_all_parameter_sets(self):
        """Get list of all available parameter sets."""
        return [(pset['name'], pset['description']) for pset in self.config['parameter_sets']]


class BehaviorAnalyzer:
    def __init__(self, num_predators, num_prey, ctrl_freq, config):
        self.num_predators = num_predators
        self.num_prey = num_prey
        self.ctrl_freq = ctrl_freq
        self.follow_threshold = config['behavior_analysis']['follow_distance_threshold']
        self.engulf_threshold = config['behavior_analysis']['engulf_distance_threshold'] 
        self.separation_threshold = config['behavior_analysis']['separation_distance_threshold']
        self.min_behavior_duration = config['behavior_analysis']['min_behavior_duration']
        self.analysis_window_sec = config.get('analysis_window_sec', 150)
        self.reset_trial()
        
    def reset_trial(self):
        self.distances = []
        self.behaviors = []
        self.current_behavior = None
        self.behavior_start_frame = 0
        self.behavior_counts = {
            'following': 0,
            'engulfing': 0,
            'separating': 0
        }
        
    def update(self, pred_positions, prey_positions, frame):
        # Compute centroids
        pred_centroid = np.mean(pred_positions, axis=0)
        prey_centroid = np.mean(prey_positions, axis=0)
        
        # Distance between centroids
        centroid_distance = np.linalg.norm(pred_centroid - prey_centroid)
        
        self.distances.append(centroid_distance)
        
        # Classify based on thresholds
        new_behavior = self._classify_behavior(centroid_distance)

        if new_behavior != self.current_behavior:
            if (self.current_behavior is not None and 
                frame - self.behavior_start_frame >= self.min_behavior_duration):
                self.behavior_counts[self.current_behavior] += 1
                self.behaviors.append({
                    'behavior': self.current_behavior,
                    'start_frame': self.behavior_start_frame,
                    'end_frame': frame,
                    'duration': frame - self.behavior_start_frame
                })
            
            self.current_behavior = new_behavior
            self.behavior_start_frame = frame
            
    def _classify_behavior(self, distance):
        if distance <= self.engulf_threshold:
            return 'engulfing'
        elif distance <= self.follow_threshold:
            return 'following'
        elif distance >= self.separation_threshold:
            return 'separating'
        else:
            return 'following'
    
    def finalize_trial(self, total_frames):
        # Finalize the last behavior
        if (self.current_behavior is not None and 
            total_frames - self.behavior_start_frame >= self.min_behavior_duration):
            self.behavior_counts[self.current_behavior] += 1
            self.behaviors.append({
                'behavior': self.current_behavior,
                'start_frame': self.behavior_start_frame,
                'end_frame': total_frames,
                'duration': total_frames - self.behavior_start_frame
            })
        
        # Calculate behavior counts for last N seconds only
        analysis_frames = int(self.analysis_window_sec * self.ctrl_freq)
        start_frame = max(0, total_frames - analysis_frames)
        
        # Re-count behaviors for the analysis window
        windowed_behavior_counts = {
            'following': 0,
            'engulfing': 0,
            'separating': 0
        }
        
        for behavior_event in self.behaviors:
            if behavior_event['end_frame'] >= start_frame:
                windowed_behavior_counts[behavior_event['behavior']] += 1
        
        self.windowed_behavior_counts = windowed_behavior_counts
    
    def get_results(self):
        return {
            'behavior_counts': self.behavior_counts.copy(),
            'windowed_behavior_counts': self.windowed_behavior_counts.copy(),
            'behaviors': self.behaviors.copy(),
            'distances': self.distances.copy(),
            'thresholds': {
                'follow': self.follow_threshold,
                'engulf': self.engulf_threshold,
                'separation': self.separation_threshold
            },
            'analysis_window_sec': self.analysis_window_sec
        }


def run_single_trial(trial_num, config, num_trials):
    """Run a single trial and return behavior analysis results."""
    
    # Extract configuration values
    NUM_PREDATORS = config['n_predators']
    NUM_PREY = config['n_prey']
    _3D = config['3d']
    BOUNDLESS = config['boundless']
    DURATION_SEC = config['duration_sec']
    DEFAULT_SIMULATION_FREQ_HZ = config['simulation']['default_simulation_freq_hz']
    DEFAULT_CONTROL_FREQ_HZ = config['simulation']['default_control_freq_hz']
    gui = config['gui']
    
    # Get flocking parameters
    flocking_params = config['flocking_parameters']
    
    # Initialize positions for this trial
    drones_ids = list(range(NUM_PREDATORS))
    
    if _3D:
        min_distance = config['initialization']['min_distance_3d']
    else:
        if BOUNDLESS:
            multiplier = config['initialization']['min_distance_formula_multiplier']
            base = config['initialization']['min_distance_formula_base']
            offset = config['initialization']['min_distance_formula_offset']
            min_distance = (np.sqrt(NUM_PREY) * multiplier * base) + offset
        else:
            min_distance = 1.3

    if BOUNDLESS:
        offset_range = config['initialization']['boundless_settings']['init_center_offset_range']
        spacing = config['initialization']['boundless_settings']['spacing']
        
        init_center_x = 3 + np.random.uniform(offset_range[0], offset_range[1])
        init_center_y = 3 + np.random.uniform(offset_range[0], offset_range[1])
        init_center_z = 3
        init_center_x_prey = init_center_x + np.random.choice([-1, 1]) * min_distance
        init_center_y_prey = init_center_y + np.random.choice([-1, 1]) * min_distance
        init_center_z_prey = init_center_z + 0
    else:
        bounded_settings = config['initialization']['bounded_settings']
        init_center_x = bounded_settings['init_center_x']
        init_center_y = bounded_settings['init_center_y']
        init_center_z = bounded_settings['init_center_z']
        init_center_x_prey = init_center_x + min_distance
        init_center_y_prey = init_center_y + min_distance
        init_center_z_prey = init_center_z + 0
        spacing = bounded_settings['spacing']

    # Initialize FlockingUtils with unified parameters
    f_util = FlockingUtils(
        NUM_PREDATORS, NUM_PREY, 
        init_center_x, init_center_y, init_center_z,
        spacing, 
        init_center_x_prey, init_center_y_prey, init_center_z_prey,
        drones_ids=drones_ids,
        boundless=BOUNDLESS, _3D=_3D,
        d_des=flocking_params.get('desired_distance'),
        sensing_range_own=flocking_params.get('sensing_range_own_swarm'),
        sensing_range_other=flocking_params.get('sensing_range_other_swarm'),
        repulsion_strength=flocking_params.get('repulsion_strength'),
        alpha_strength=flocking_params.get('alpha'),
        epsilon_strength=flocking_params.get('epsilon'),
        adaptive_sigma_strength=flocking_params.get('adaptive_sigma_strength')
    )

    # Initialize positions
    if _3D:
        pos_xs, pos_ys, pos_zs, pos_hxs, pos_hys, pos_hzs = f_util.initialize_positions3D(preds=True)
        pos_xs_prey, pos_ys_prey, pos_zs_prey, pos_h_xc_prey, pos_h_yc_prey, pos_h_zc_prey = f_util.initialize_positions3D(preds=False)
    else:
        pos_xs, pos_ys, pos_zs, pos_h_xc, pos_h_yc, pos_h_zc = f_util.initialize_positions(is_predator=True)
        pos_xs_prey, pos_ys_prey, pos_zs_prey, pos_h_xc_prey, pos_h_yc_prey, pos_h_zc_prey = f_util.initialize_positions(is_predator=False)

    INIT_XYZ_PREDATORS = np.zeros([NUM_PREDATORS, 3])
    INIT_XYZ_PREDATORS[:, 0] = pos_xs
    INIT_XYZ_PREDATORS[:, 1] = pos_ys
    INIT_XYZ_PREDATORS[:, 2] = pos_zs
    INIT_RPY_PREDATORS = np.array([[.0, .0, .0] for _ in range(NUM_PREDATORS)])

    INIT_XYZ_PREY = np.zeros([NUM_PREY, 3])
    INIT_XYZ_PREY[:, 0] = pos_xs_prey
    INIT_XYZ_PREY[:, 1] = pos_ys_prey
    INIT_XYZ_PREY[:, 2] = pos_zs_prey
    INIT_RPY_PREY = np.array([[.0, .0, .0] for _ in range(NUM_PREY)])

    # Create environment
    env = CtrlAviary_(drone_model=DroneModel("cf2x"),
                     num_drones=NUM_PREDATORS,
                     preys=NUM_PREY,
                     initial_xyzs=INIT_XYZ_PREDATORS,
                     initial_rpys=INIT_RPY_PREDATORS,
                     initial_xyzs_preys=INIT_XYZ_PREY,
                     initial_rpys_preys=INIT_RPY_PREY,
                     physics=Physics("pyb"),
                     record=False,
                     neighbourhood_radius=10,
                     pyb_freq=DEFAULT_SIMULATION_FREQ_HZ,
                     ctrl_freq=DEFAULT_CONTROL_FREQ_HZ,
                     gui=gui,
                     user_debug_gui=gui)
    
    # Color agents if GUI is active
    if gui:
        PYB_CLIENT = env.getPyBulletClient()
        Predators_ids = env.getDroneIds()
        Prey_ids = list(range(NUM_PREDATORS, NUM_PREDATORS + NUM_PREY))

        # Predators: Red
        red = [1.0, 0.0, 0.0, 1.0] 
        for drone_id in Predators_ids:
            p.changeVisualShape(objectUniqueId=drone_id, 
                                linkIndex=-1, 
                                rgbaColor=red, 
                                physicsClientId=PYB_CLIENT)

        # Prey: Blue
        blue = [0.0, 0.0, 1.0, 1.0] 
        for prey_id in Prey_ids:
            p.changeVisualShape(objectUniqueId=prey_id, 
                                linkIndex=-1, 
                                rgbaColor=blue, 
                                physicsClientId=PYB_CLIENT)

    # Controllers
    ctrl_predators = [DSLPIDControl(drone_model=DroneModel("cf2x")) for i in range(NUM_PREDATORS)]
    ctrl_prey = [DSLPIDControl(drone_model=DroneModel("cf2x")) for i in range(NUM_PREY)]

    # Initialize behavior analyzer
    analyzer = BehaviorAnalyzer(NUM_PREDATORS, NUM_PREY, DEFAULT_CONTROL_FREQ_HZ, config)

    action_predators = np.zeros((NUM_PREDATORS, 4))
    action_prey = np.zeros((NUM_PREY, 4))

    pos_x = np.zeros(NUM_PREDATORS)
    pos_y = np.zeros(NUM_PREDATORS)
    pos_z = np.zeros(NUM_PREDATORS)
    pos_x_prey = np.zeros(NUM_PREY)
    pos_y_prey = np.zeros(NUM_PREY)
    pos_z_prey = np.zeros(NUM_PREY)

    total_frames = int(DURATION_SEC * env.CTRL_FREQ)
    
    parameter_set_name = config.get('_parameter_set_name', 'default')
    print(f"Running trial {trial_num + 1}/{num_trials} with parameter set '{parameter_set_name}'...")

    for i in range(total_frames):
        obs, obs_prey, reward, reward_prey, done, done_prey, info, info_prey, _, _ = env.step(action_predators, action_prey)
        
        # Get positions
        for j, k in zip(range(NUM_PREDATORS), range(NUM_PREY)):
            states = env._getDroneStateVector(j)
            states_prey = env._getDroneStateVectorPreys(k)
            pos_x[j] = states[0]
            pos_y[j] = states[1]
            pos_z[j] = states[2]
            pos_x_prey[k] = states_prey[0]
            pos_y_prey[k] = states_prey[1]
            pos_z_prey[k] = states_prey[2]

        # Analyze behavior
        pred_positions = np.column_stack([pos_x, pos_y, pos_z])
        prey_positions = np.column_stack([pos_x_prey, pos_y_prey, pos_z_prey])
        analyzer.update(pred_positions, prey_positions, i)

        # Calculate forces and update positions
        f_util.calc_dij(pos_x, pos_y, pos_z, pos_x_prey, pos_y_prey, pos_z_prey)
        f_util.calc_ang_ij(pos_x, pos_y, pos_z, pos_x_prey, pos_y_prey, pos_z_prey)
        f_util.calc_grad_vals(pos_x, pos_y, pos_z, pos_x_prey, pos_y_prey, pos_z_prey, _3D)
        f_util.calc_p_forces()  # Prey forces
        f_util.calc_p_forcesADM()  # Predator forces
        f_util.calc_repulsion_predator_forces(pos_x, pos_y, pos_z, pos_x_prey, pos_y_prey, pos_z_prey)

        u, u_prey = f_util.calc_u_w()
        pos_hxs, pos_hys, pos_hzs, pos_h_xc_prey, pos_h_yc_prey, pos_h_zc_prey = f_util.get_heading()
        
        if not BOUNDLESS:
            f_util.calc_boun_rep(pos_x, pos_y, pos_z, pos_hxs, pos_hys, pos_hzs, preds=True)
            f_util.calc_boun_rep(pos_x_prey, pos_y_prey, pos_z_prey, pos_h_xc_prey, pos_h_yc_prey, pos_h_zc_prey, preds=False)
        
        f_util.update_heading()

        # Control actions
        if _3D:
            for j, k in zip(range(NUM_PREDATORS), range(NUM_PREY)):
                vel_cmd = np.array([u[j]*np.cos(pos_hxs[j]), u[j]*np.cos(pos_hys[j]), u[j]*np.cos(pos_hzs[j])])
                pos_cmd = np.array([pos_x[j], pos_y[j], pos_z[j]])
                action_predators[j], _, _ = ctrl_predators[j].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs[j], target_pos=pos_cmd,
                    target_vel=vel_cmd, target_rpy=np.array([0, 0, 0]))
                
                vel_cmd_prey = np.array([u_prey[k]*np.cos(pos_h_xc_prey[k]), u_prey[k]*np.cos(pos_h_yc_prey[k]), u_prey[k]*np.cos(pos_h_zc_prey[k])])
                pos_cmd_prey = np.array([pos_x_prey[k], pos_y_prey[k], pos_z_prey[k]])
                action_prey[k], _, _ = ctrl_prey[k].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs_prey[k], target_pos=pos_cmd_prey,
                    target_vel=vel_cmd_prey, target_rpy=np.array([0, 0, 0]))
        else:
            for j, k in zip(range(NUM_PREDATORS), range(NUM_PREY)):
                vel_cmd = np.array([u[j] * np.cos(pos_hxs[j]), u[j] * np.cos(pos_hys[j]), 0])
                pos_cmd = np.array([pos_x[j], pos_y[j], pos_z[j]])
                action_predators[j], _, _ = ctrl_predators[j].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs[j], target_pos=pos_cmd,
                    target_vel=vel_cmd, target_rpy=np.array([0, 0, 0]))
                
                vel_cmd_prey = np.array([u_prey[k] * np.cos(pos_h_xc_prey[k]), u_prey[k] * np.cos(pos_h_yc_prey[k]), 0])
                pos_cmd_prey = np.array([pos_x_prey[k], pos_y_prey[k], pos_z_prey[k]])
                action_prey[k], _, _ = ctrl_prey[k].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs_prey[k], target_pos=pos_cmd_prey,
                    target_vel=vel_cmd_prey, target_rpy=np.array([0, 0, 0]))

        if gui:
            # Calculate midpoint for camera
            midpoint_x = (np.sum(pos_x) + np.sum(pos_x_prey)) / (NUM_PREDATORS + NUM_PREY)
            midpoint_y = (np.sum(pos_y) + np.sum(pos_y_prey)) / (NUM_PREDATORS + NUM_PREY)
            
            # Top-down view
            p.resetDebugVisualizerCamera(cameraDistance=7.5,
                                         cameraYaw=0, 
                                         cameraPitch=-89.9,
                                         cameraTargetPosition=[midpoint_x, midpoint_y, 0])
            env.render()

    # Finalize analysis
    analyzer.finalize_trial(total_frames)
    
    # Close environment
    env.close()
    
    return analyzer.get_results()


def run_multiple_trials(config):
    """Run multiple trials and collect results."""
    all_results = []
    num_trials = config['num_trials']
    
    start_time = time.time()
    
    for trial in range(num_trials):
        trial_results = run_single_trial(trial, config, num_trials)
        trial_results['trial'] = trial
        all_results.append(trial_results)
    
    end_time = time.time()
    parameter_set_name = config.get('_parameter_set_name', 'default')
    print(f"\nCompleted {num_trials} trials for parameter set '{parameter_set_name}' in {end_time - start_time:.2f} seconds")
    
    return all_results


def analyze_results(all_results, config):
    """Analyze and summarize results across all trials."""
    
    # Collect windowed behavior counts
    following_counts = [r['windowed_behavior_counts']['following'] for r in all_results]
    engulfing_counts = [r['windowed_behavior_counts']['engulfing'] for r in all_results]
    separating_counts = [r['windowed_behavior_counts']['separating'] for r in all_results]
    
    # Calculate statistics
    stats = {
        'following': {
            'mean': np.mean(following_counts),
            'std': np.std(following_counts),
            'min': np.min(following_counts),
            'max': np.max(following_counts),
            'total': np.sum(following_counts)
        },
        'engulfing': {
            'mean': np.mean(engulfing_counts),
            'std': np.std(engulfing_counts),
            'min': np.min(engulfing_counts),
            'max': np.max(engulfing_counts),
            'total': np.sum(engulfing_counts)
        },
        'separating': {
            'mean': np.mean(separating_counts),
            'std': np.std(separating_counts),
            'min': np.min(separating_counts),
            'max': np.max(separating_counts),
            'total': np.sum(separating_counts)
        }
    }
    
    # Print summary
    parameter_set_name = config.get('_parameter_set_name', 'default')
    parameter_set_desc = config.get('_parameter_set_description', 'Default configuration')
    analysis_window = config.get('analysis_window_sec', 150)
    
    print("\n" + "="*70)
    print(f"BEHAVIOR ANALYSIS SUMMARY - {parameter_set_name.upper()}")
    print("="*70)
    print(f"Description: {parameter_set_desc}")
    print(f"Number of trials: {len(all_results)}")
    print(f"Duration per trial: {config['duration_sec']} seconds")
    print(f"Analysis window: Last {analysis_window} seconds")
    print(f"Number of predators/prey: {config['n_predators']}/{config['n_prey']}")
    print(f"Environment: {'3D' if config['3d'] else '2D'}, {'Boundless' if config['boundless'] else 'Bounded'}")
    
    flocking_params = config['flocking_parameters']
    print(f"\nFlocking Parameters:")
    print(f"  Desired Distance (d_des): {flocking_params.get('desired_distance', 'N/A')}")
    print(f"  Sigma (σ = d_des/√2): {flocking_params.get('desired_distance', 0) / np.sqrt(2):.3f}")
    print(f"  Sensing Range (Own Swarm): {flocking_params.get('sensing_range_own_swarm', 'N/A')}m")
    print(f"  Sensing Range (Other Swarm): {flocking_params.get('sensing_range_other_swarm', 'N/A')}m")
    print(f"  Repulsion Strength: {flocking_params.get('repulsion_strength', 'N/A')}")
    print(f"  Alpha: {flocking_params.get('alpha', 'N/A')}")
    print(f"  Epsilon: {flocking_params.get('epsilon', 'N/A')}")
    
    print(f"\nBehavior Thresholds:")
    print(f"  Follow: {config['behavior_analysis']['follow_distance_threshold']}m")
    print(f"  Engulf: {config['behavior_analysis']['engulf_distance_threshold']}m")
    print(f"  Separation: {config['behavior_analysis']['separation_distance_threshold']}m")
    print()
    
    for behavior, data in stats.items():
        print(f"{behavior.upper()} (Last {analysis_window}s):")
        print(f"  Total occurrences: {data['total']}")
        print(f"  Mean per trial: {data['mean']:.2f} ± {data['std']:.2f}")
        print(f"  Range: {data['min']} - {data['max']}")
        print()
    
    return stats, all_results


def create_simplified_visualizations(stats, all_results, config, save_dir):
    """Create essential plots: bar chart, individual trials, and mean ± std."""
    
    parameter_set_name = config.get('_parameter_set_name', 'default')
    control_freq = config['simulation']['default_control_freq_hz']
    thresholds = all_results[0]['thresholds']
    analysis_window = config.get('analysis_window_sec', 150)
    
    # Create figure with 1 row and 3 columns
    fig, axes = plt.subplots(1, 3, figsize=(20, 6))
    fig.suptitle(f'Predator-Prey Behavior Analysis - {parameter_set_name} (Last {analysis_window}s)', 
                 fontsize=16, fontweight='bold')
    
    # 1. Bar chart of mean behavior counts
    behaviors = list(stats.keys())
    means = [stats[b]['mean'] for b in behaviors]
    stds = [stats[b]['std'] for b in behaviors]
    
    ax1 = axes[0]
    bars = ax1.bar(behaviors, means, yerr=stds, capsize=5, color=['skyblue', 'lightcoral', 'lightgreen'])
    ax1.set_title(f'Mean Behavior Counts per Trial\n(Last {analysis_window}s)')
    ax1.set_ylabel('Count')
    ax1.grid(axis='y', alpha=0.3)
    
    for i, (bar, mean, std) in enumerate(zip(bars, means, stds)):
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + std + 0.1,
                f'{mean:.1f}', ha='center', va='bottom')
    
    # 2. Distance Evolution for ALL Trials
    ax2 = axes[1]
    if all_results:
        colors = plt.cm.tab10(np.linspace(0, 1, len(all_results)))
        
        for i, result in enumerate(all_results):
            distances = result['distances']
            time_steps = np.arange(len(distances)) / control_freq
            ax2.plot(time_steps, distances, alpha=0.6, linewidth=1, 
                    color=colors[i], label=f'Trial {i+1}')
        
        # Add threshold lines
        ax2.axhline(y=thresholds['follow'], color='orange', linestyle='--', linewidth=2,
                   label=f'Follow ({thresholds["follow"]}m)')
        ax2.axhline(y=thresholds['engulf'], color='red', linestyle='--', linewidth=2,
                   label=f'Engulf ({thresholds["engulf"]}m)')
        ax2.axhline(y=thresholds['separation'], color='green', linestyle='--', linewidth=2,
                   label=f'Separation ({thresholds["separation"]}m)')
        
        # Highlight analysis window
        total_time = len(all_results[0]['distances']) / control_freq
        analysis_start_time = total_time - analysis_window
        ax2.axvspan(analysis_start_time, total_time, alpha=0.1, color='gray', 
                   label=f'Analysis Window ({analysis_window}s)')
        
        ax2.set_title('Distance Evolution - All Trials')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Centroid Distance (m)')
        ax2.grid(alpha=0.3)
        
        # Show threshold and analysis window in legend
        handles, labels = ax2.get_legend_handles_labels()
        important_handles = handles[-4:]
        important_labels = labels[-4:]
        ax2.legend(important_handles, important_labels, loc='upper right', fontsize=8)
    
    # 3. Distance Evolution - Statistical Summary (Mean ± Std)
    ax3 = axes[2]
    if all_results:
        min_length = min(len(result['distances']) for result in all_results)
        aligned_distances = np.array([result['distances'][:min_length] for result in all_results])
        
        mean_distances = np.mean(aligned_distances, axis=0)
        std_distances = np.std(aligned_distances, axis=0)
        time_steps = np.arange(min_length) / control_freq
        
        ax3.fill_between(time_steps, 
                        mean_distances - std_distances,
                        mean_distances + std_distances,
                        alpha=0.3, color='blue', label='±1 Std Dev')
        ax3.plot(time_steps, mean_distances, 'b-', linewidth=2, label='Mean Distance')
        
        ax3.axhline(y=thresholds['follow'], color='orange', linestyle='--', linewidth=2,
                   label=f'Follow ({thresholds["follow"]}m)')
        ax3.axhline(y=thresholds['engulf'], color='red', linestyle='--', linewidth=2,
                   label=f'Engulf ({thresholds["engulf"]}m)')
        ax3.axhline(y=thresholds['separation'], color='green', linestyle='--', linewidth=2,
                   label=f'Separation ({thresholds["separation"]}m)')
        
        # Highlight analysis window
        total_time = min_length / control_freq
        analysis_start_time = total_time - analysis_window
        ax3.axvspan(analysis_start_time, total_time, alpha=0.1, color='gray',
                   label=f'Analysis Window ({analysis_window}s)')
        
        ax3.set_title('Distance Evolution - Mean ± Std')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Centroid Distance (m)')
        ax3.legend(fontsize=8)
        ax3.grid(alpha=0.3)
    
    # Save the plot
    plt.tight_layout()
    filename = f'behavior_analysis_{parameter_set_name}.png'
    plt.savefig(os.path.join(save_dir, filename), dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved: {filename}")


def save_results(stats, all_results, config, save_dir):
    """Save analysis results to files including raw data for re-plotting."""
    
    parameter_set_name = config.get('_parameter_set_name', 'default')
    
    # Helper function to convert numpy types to Python types
    def convert_to_python_types(obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {key: convert_to_python_types(value) for key, value in obj.items()}
        elif isinstance(obj, list):
            return [convert_to_python_types(item) for item in obj]
        else:
            return obj
    
    # Convert all data to Python types
    stats_python = convert_to_python_types(stats)
    all_results_python = convert_to_python_types(all_results)
    
    # Save configuration used for this experiment
    config_file = os.path.join(save_dir, f'config_{parameter_set_name}.json')
    with open(config_file, 'w') as f:
        config_to_save = deepcopy(config)
        config_to_save.pop('_parameter_set_name', None)
        config_to_save.pop('_parameter_set_description', None)
        json.dump(config_to_save, f, indent=2)
    
    # Save statistics as JSON
    stats_file = os.path.join(save_dir, f'behavior_statistics_{parameter_set_name}.json')
    with open(stats_file, 'w') as f:
        json.dump(stats_python, f, indent=2)
    
    # Save detailed results as JSON
    results_file = os.path.join(save_dir, f'detailed_results_{parameter_set_name}.json')
    with open(results_file, 'w') as f:
        json.dump(all_results_python, f, indent=2)
    
    # Save raw data for plotting (PICKLE format)
    plot_data = {
        'stats': stats,
        'all_results': all_results,
        'config': config,
        'parameter_set_name': parameter_set_name
    }
    plot_data_file = os.path.join(save_dir, f'plot_data_{parameter_set_name}.pkl')
    with open(plot_data_file, 'wb') as f:
        pickle.dump(plot_data, f)
    
    # Save CSV data for easy sharing
    csv_data_file = os.path.join(save_dir, f'distance_data_{parameter_set_name}.csv')
    distance_data = []
    
    for trial_idx, result in enumerate(all_results):
        distances = result['distances']
        control_freq = config['simulation']['default_control_freq_hz']
        
        for frame_idx, distance in enumerate(distances):
            distance_data.append({
                'trial': trial_idx + 1,
                'frame': frame_idx,
                'time_sec': frame_idx / control_freq,
                'distance': float(distance)
            })
    
    distance_df = pd.DataFrame(distance_data)
    distance_df.to_csv(csv_data_file, index=False)
    
    # Save summary as CSV
    summary_data = []
    for i, result in enumerate(all_results):
        row = {
            'parameter_set': parameter_set_name,
            'trial': i + 1,
            'following_count_windowed': int(result['windowed_behavior_counts']['following']),
            'engulfing_count_windowed': int(result['windowed_behavior_counts']['engulfing']),
            'separating_count_windowed': int(result['windowed_behavior_counts']['separating']),
            'following_count_full': int(result['behavior_counts']['following']),
            'engulfing_count_full': int(result['behavior_counts']['engulfing']),
            'separating_count_full': int(result['behavior_counts']['separating']),
            'total_behaviors_windowed': int(sum(result['windowed_behavior_counts'].values())),
            'total_behaviors_full': int(sum(result['behavior_counts'].values())),
            'avg_distance': float(np.mean(result['distances'])),
            'min_distance': float(np.min(result['distances'])),
            'max_distance': float(np.max(result['distances'])),
            'std_distance': float(np.std(result['distances'])),
            'follow_threshold': float(result['thresholds']['follow']),
            'engulf_threshold': float(result['thresholds']['engulf']),
            'separation_threshold': float(result['thresholds']['separation']),
            'analysis_window_sec': int(result['analysis_window_sec'])
        }
        summary_data.append(row)
    
    summary_df = pd.DataFrame(summary_data)
    summary_file = os.path.join(save_dir, f'trial_summary_{parameter_set_name}.csv')
    summary_df.to_csv(summary_file, index=False)
    
    # Save simplified TXT file with 1 Hz sampling
    txt_file = os.path.join(save_dir, f'distance_data_{parameter_set_name}.txt')
    with open(txt_file, 'w') as f:
        f.write("trial\ttime_sec\tdistance\n")
        for trial_idx, result in enumerate(all_results):
            distances = result['distances']
            control_freq = config['simulation']['default_control_freq_hz']
            total_time = len(distances) // control_freq
            
            for t in range(total_time + 1):
                frame_idx = t * control_freq
                if frame_idx < len(distances):
                    f.write(f"{trial_idx+1}\t{t}\t{distances[frame_idx]:.6f}\n")

    # Save metadata JSON
    metadata = {
        'parameter_set_name': parameter_set_name,
        'description': config.get('_parameter_set_description', ''),
        'num_trials': int(config['num_trials']),
        'duration_sec': int(config['duration_sec']),
        'analysis_window_sec': int(config.get('analysis_window_sec', 150)),
        'n_predators': int(config['n_predators']),
        'n_prey': int(config['n_prey']),
        'control_freq_hz': int(config['simulation']['default_control_freq_hz']),
        'flocking_parameters': convert_to_python_types(config['flocking_parameters']),
        'behavior_thresholds': convert_to_python_types(config['behavior_analysis'])
    }
    
    metadata_file = os.path.join(save_dir, f'metadata_{parameter_set_name}.json')
    with open(metadata_file, 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"\nResults saved to:")
    print(f"  - Configuration: {config_file}")
    print(f"  - Statistics: {stats_file}")
    print(f"  - Detailed results: {results_file}")
    print(f"  - Plot data: {plot_data_file}")
    print(f"  - Distance CSV: {csv_data_file}")
    print(f"  - Summary CSV: {summary_file}")
    print(f"  - Metadata: {metadata_file}")


def load_and_plot(results_dir):
    """Load saved plot data and regenerate plots without running simulation."""
    
    # Find the plot data file
    plot_data_files = [f for f in os.listdir(results_dir) if f.startswith('plot_data_') and f.endswith('.pkl')]
    
    if not plot_data_files:
        print(f"Error: No plot data files found in {results_dir}")
        return
    
    # Load the first plot data file found
    plot_data_file = os.path.join(results_dir, plot_data_files[0])
    print(f"Loading plot data from: {plot_data_file}")
    
    with open(plot_data_file, 'rb') as f:
        plot_data = pickle.load(f)
    
    stats = plot_data['stats']
    all_results = plot_data['all_results']
    config = plot_data['config']
    
    print(f"Loaded data for parameter set: {config.get('_parameter_set_name', 'unknown')}")
    print(f"Number of trials: {len(all_results)}")
    
    # Regenerate plots
    create_simplified_visualizations(stats, all_results, config, results_dir)
    
    print(f"\nPlots regenerated and saved to: {results_dir}")


def run_experiment_with_parameter_set(config_manager, parameter_set_name, base_save_dir):
    """Run a complete experiment with a specific parameter set."""
    
    # Get configuration for this parameter set
    config = config_manager.get_parameter_set_config(parameter_set_name)
    
    parameter_set_name = config.get('_parameter_set_name', 'default')
    print(f"\n{'='*80}")
    print(f"RUNNING EXPERIMENT: {parameter_set_name.upper()}")
    print(f"Description: {config.get('_parameter_set_description', 'N/A')}")
    print(f"{'='*80}")
    
    # Run trials
    all_results = run_multiple_trials(config)
    
    # Analyze results
    stats, all_results = analyze_results(all_results, config)
    
    # Create timestamped save directory
    dt_string = datetime.now().strftime("%d-%m-%Y_%H-%M")
    save_dir = os.path.join(base_save_dir, f"results_{parameter_set_name}_{dt_string}")
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    # Create visualizations
    create_simplified_visualizations(stats, all_results, config, save_dir)
    
    # Save results
    save_results(stats, all_results, config, save_dir)
    
    print(f"\nExperiment '{parameter_set_name}' completed!")
    print(f"Results saved to: {save_dir}")
    
    return {
        'parameter_set_name': parameter_set_name,
        'config': config,
        'stats': stats,
        'all_results': all_results,
        'save_dir': save_dir
    }


def main():
    """Main function to run experiments with different parameter sets."""
    
    parser = argparse.ArgumentParser(description='Run predator-prey behavior analysis with unified symmetric parameters')
    parser.add_argument('--config', type=str, default='config.json',
                       help='Path to configuration JSON file (default: config.json)')
    parser.add_argument('--parameter_set', type=str, default=None,
                       help='Name of specific parameter set to run (default: run default configuration)')
    parser.add_argument('--run_all_sets', action='store_true',
                       help='Run experiments with all parameter sets defined in config')
    parser.add_argument('--list_sets', action='store_true',
                       help='List available parameter sets and exit')
    parser.add_argument('--output_dir', type=str, default='experiment_results',
                       help='Base output directory for results (default: experiment_results)')
    parser.add_argument('--plot_only', action='store_true',
                       help='Only regenerate plots from saved data (no simulation)')
    parser.add_argument('--results_dir', type=str, default=None,
                       help='Directory containing saved plot data for --plot_only mode')
    
    args = parser.parse_args()
    
    # Plot-only mode
    if args.plot_only:
        if not args.results_dir:
            print("Error: --results_dir required for --plot_only mode")
            print("Example: python run2.py --plot_only --results_dir experiment_results/results_default_config_02-10-2025_14-30")
            sys.exit(1)
        
        if not os.path.exists(args.results_dir):
            print(f"Error: Results directory not found: {args.results_dir}")
            sys.exit(1)
        
        load_and_plot(args.results_dir)
        return
    
    # Initialize configuration manager
    config_manager = ConfigManager(args.config)
    
    # List parameter sets if requested
    if args.list_sets:
        parameter_sets = config_manager.get_all_parameter_sets()
        print("\nAvailable parameter sets:")
        print("-" * 50)
        for name, description in parameter_sets:
            print(f"  {name:<25} - {description}")
        print()
        return
    
    # Create base output directory
    base_save_dir = args.output_dir
    if not os.path.exists(base_save_dir):
        os.makedirs(base_save_dir)
    
    print("Predator-Prey Multi-Trial Analysis with Unified Symmetric Parameters")
    print(f"Configuration file: {args.config}")
    print(f"Output directory: {base_save_dir}")
    
    all_experiment_results = []
    
    if args.run_all_sets:
        # Run experiments with all parameter sets
        parameter_sets = config_manager.get_all_parameter_sets()
        print(f"\nRunning experiments with {len(parameter_sets)} parameter sets...")
        
        for parameter_set_name, _ in parameter_sets:
            experiment_result = run_experiment_with_parameter_set(
                config_manager, parameter_set_name, base_save_dir)
            all_experiment_results.append(experiment_result)
    
    else:
        # Run experiment with single parameter set
        parameter_set_name = args.parameter_set
        experiment_result = run_experiment_with_parameter_set(
            config_manager, parameter_set_name, base_save_dir)
        all_experiment_results.append(experiment_result)
    
    # Final summary
    print(f"\n{'='*80}")
    print("EXPERIMENT SUMMARY")
    print(f"{'='*80}")
    print(f"Total experiments completed: {len(all_experiment_results)}")
    print(f"Results saved to: {base_save_dir}")
    
    print("\nTo regenerate plots from saved data (without re-running simulation):")
    for exp_result in all_experiment_results:
        print(f"  python {sys.argv[0]} --plot_only --results_dir {exp_result['save_dir']}")
    
    print("\nTo run specific experiments:")
    print(f"  python {sys.argv[0]} --config {args.config} --list_sets")
    print(f"  python {sys.argv[0]} --config {args.config} --parameter_set <name>")
    print(f"  python {sys.argv[0]} --config {args.config} --run_all_sets")


if __name__ == "__main__":
    main()