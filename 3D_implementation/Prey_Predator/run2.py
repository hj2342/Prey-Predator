# """CrazyFlie software-in-the-loop control example with multi-trial analysis.

# Setup
# -----
# Step 1: Clone pycffirmware from https://github.com/utiasDSL/pycffirmware
# Step 2: Follow the install instructions for pycffirmware in its README

# Example
# -------
# In terminal, run:
# python gym_pybullet_drones/examples/cf_multi_trial.py

# """

# import os
# import sys

# current_dir = os.path.dirname(os.path.realpath(__file__))
# parent_dir = os.path.dirname(current_dir)
# sys.path.insert(0, parent_dir)

# import time
# import argparse
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
# import json

# # # Configuration
# NUM_TRIALS = 1  # Number of trials to run
# BOUNDLESS = True
# if BOUNDLESS:
#     NS = 10
#     PERC_NO_SENSOR = 0
# else:
#     NS = 5
#     PERC_NO_SENSOR = 0

# _3D = False
# PLOT = False  # Disable plotting during background runs
# GUI = False   # Disable GUI for background runs
# self_log = True

# # Simulation parameters
# DEFAULT_DRONES = DroneModel("cf2x")
# DEFAULT_PHYSICS = Physics("pyb")
# DEFAULT_SIMULATION_FREQ_HZ = 240
# DEFAULT_CONTROL_FREQ_HZ = 48
# DEFAULT_OUTPUT_FOLDER = 'results'
# DURATION_SEC =100  # Reduced for faster trials

# NUM_DRONES = NS
# PREYS = NS

# # Behavior analysis parameters
# FOLLOW_DISTANCE_THRESHOLD = 2.0     # Distance threshold for "following"
# ENGULF_DISTANCE_THRESHOLD = 1.0     # Distance threshold for "engulfing"
# SEPARATION_DISTANCE_THRESHOLD = 4.0  # Distance threshold for "separation"
# MIN_BEHAVIOR_DURATION = 5           # Minimum frames to count as a behavior

# class BehaviorAnalyzer:
#     def __init__(self, num_predators, num_preys, ctrl_freq):
#         self.num_predators = num_predators
#         self.num_preys = num_preys
#         self.ctrl_freq = ctrl_freq
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
#         # Calculate average distances between predators and preys
#         min_distances = []
#         for i in range(self.num_predators):
#             pred_pos = pred_positions[i]
#             min_dist = float('inf')
#             for j in range(self.num_preys):
#                 prey_pos = prey_positions[j]
#                 dist = np.linalg.norm(pred_pos - prey_pos)
#                 min_dist = min(min_dist, dist)
#             min_distances.append(min_dist)
        
#         avg_min_distance = np.mean(min_distances)
#         self.distances.append(avg_min_distance)
        
#         # Determine current behavior
#         new_behavior = self._classify_behavior(avg_min_distance)
        
#         # Check for behavior changes
#         if new_behavior != self.current_behavior:
#             # End previous behavior if it lasted long enough
#             if (self.current_behavior is not None and 
#                 frame - self.behavior_start_frame >= MIN_BEHAVIOR_DURATION):
#                 self.behavior_counts[self.current_behavior] += 1
#                 self.behaviors.append({
#                     'behavior': self.current_behavior,
#                     'start_frame': self.behavior_start_frame,
#                     'end_frame': frame,
#                     'duration': frame - self.behavior_start_frame
#                 })
            
#             # Start new behavior
#             self.current_behavior = new_behavior
#             self.behavior_start_frame = frame
    
#     def _classify_behavior(self, distance):
#         if distance <= ENGULF_DISTANCE_THRESHOLD:
#             return 'engulfing'
#         elif distance <= FOLLOW_DISTANCE_THRESHOLD:
#             return 'following'
#         elif distance >= SEPARATION_DISTANCE_THRESHOLD:
#             return 'separating'
#         else:
#             return 'following'  # Default to following for intermediate distances
    
#     def finalize_trial(self, total_frames):
#         # Finalize the last behavior
#         if (self.current_behavior is not None and 
#             total_frames - self.behavior_start_frame >= MIN_BEHAVIOR_DURATION):
#             self.behavior_counts[self.current_behavior] += 1
#             self.behaviors.append({
#                 'behavior': self.current_behavior,
#                 'start_frame': self.behavior_start_frame,
#                 'end_frame': total_frames,
#                 'duration': total_frames - self.behavior_start_frame
#             })
    
#     def get_results(self):
#         return {
#             'behavior_counts': self.behavior_counts.copy(),
#             'behaviors': self.behaviors.copy(),
#             'distances': self.distances.copy()
#         }

# def run_single_trial(trial_num, duration_sec=DURATION_SEC, gui=False, plot=False):
#     """Run a single trial and return behavior analysis results."""
    
#     # Initialize positions for this trial
#     drones_ids = list(range(NUM_DRONES))
    
#     if _3D:
#         min_distance = 3.
#     else:
#         if BOUNDLESS:
#             min_distance = (np.sqrt(PREYS) * 1.7 * 0.3) + 3/1.5
#         else:
#             min_distance = 1.3

#     if BOUNDLESS:
#         init_center_x = 3 + np.random.uniform(-1, 1)  # Add randomness
#         init_center_y = 3 + np.random.uniform(-1, 1)
#         init_center_z = 3
#         init_center_x_preys = init_center_x + np.random.choice([-1, 1]) * min_distance
#         init_center_y_preys = init_center_y + np.random.choice([-1, 1]) * min_distance
#         init_center_z_preys = init_center_z + 0
#         spacing = 0.95
#     else:
#         init_center_x = 1
#         init_center_y = 1
#         init_center_z = 2
#         init_center_x_preys = init_center_x + min_distance
#         init_center_y_preys = init_center_y + min_distance
#         init_center_z_preys = init_center_z + 0
#         spacing = 0.2

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
#     env = CtrlAviary_(drone_model=DEFAULT_DRONES,
#                      num_drones=NUM_DRONES,
#                      preys=PREYS,
#                      initial_xyzs=INIT_XYZ,
#                      initial_rpys=INIT_RPY,
#                      initial_xyzs_preys=INIT_XYZ_PREYS,
#                      initial_rpys_preys=INIT_RPY_PREYS,
#                      physics=DEFAULT_PHYSICS,
#                      record=False,
#                      neighbourhood_radius=10,
#                      pyb_freq=DEFAULT_SIMULATION_FREQ_HZ,
#                      ctrl_freq=DEFAULT_CONTROL_FREQ_HZ,
#                      gui=gui,
#                      user_debug_gui=gui)

#     # Controllers
#     ctrl = [DSLPIDControl(drone_model=DEFAULT_DRONES) for i in range(NUM_DRONES)]
#     ctrl_preys = [DSLPIDControl(drone_model=DEFAULT_DRONES) for i in range(PREYS)]

#     # Initialize behavior analyzer
#     analyzer = BehaviorAnalyzer(NUM_DRONES, PREYS, DEFAULT_CONTROL_FREQ_HZ)

#     START = time.time() # remove this and use simulated seconds
#     action = np.zeros((NUM_DRONES, 4))
#     action_preys = np.zeros((PREYS, 4))

#     pos_x = np.zeros(NUM_DRONES)
#     pos_y = np.zeros(NUM_DRONES)
#     pos_z = np.zeros(NUM_DRONES)
#     pos_x_preys = np.zeros(PREYS)
#     pos_y_preys = np.zeros(PREYS)
#     pos_z_preys = np.zeros(PREYS)

#     total_frames = int(duration_sec * env.CTRL_FREQ)
    
#     print(f"Running trial {trial_num + 1}/{NUM_TRIALS}...")

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
#             env.render()
#             sync(i, START, env.CTRL_TIMESTEP)

#     # Finalize analysis
#     analyzer.finalize_trial(total_frames)
    
#     # Close environment
#     env.close()
    
#     return analyzer.get_results()

# def run_multiple_trials(num_trials=NUM_TRIALS):
#     """Run multiple trials and collect results."""
#     all_results = []
    
#     start_time = time.time()
    
#     for trial in range(num_trials):
#         trial_results = run_single_trial(trial, duration_sec=DURATION_SEC, gui=False, plot=False)
#         trial_results['trial'] = trial
#         all_results.append(trial_results)
    
#     end_time = time.time()
#     print(f"\nCompleted {num_trials} trials in {end_time - start_time:.2f} seconds")
    
#     return all_results

# def analyze_results(all_results):
#     """Analyze and summarize results across all trials."""
    
#     # Collect behavior counts
#     following_counts = [r['behavior_counts']['following'] for r in all_results]
#     engulfing_counts = [r['behavior_counts']['engulfing'] for r in all_results]
#     separating_counts = [r['behavior_counts']['separating'] for r in all_results]
    
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
#     print("\n" + "="*50)
#     print("BEHAVIOR ANALYSIS SUMMARY")
#     print("="*50)
#     print(f"Number of trials: {len(all_results)}")
#     print(f"Duration per trial: {DURATION_SEC} seconds")
#     print()
    
#     for behavior, data in stats.items():
#         print(f"{behavior.upper()}:")
#         print(f"  Total occurrences: {data['total']}")
#         print(f"  Mean per trial: {data['mean']:.2f} ± {data['std']:.2f}")
#         print(f"  Range: {data['min']} - {data['max']}")
#         print()
    
#     return stats, all_results
# def create_visualizations_with_all_distances(stats, all_results):
#     """Create visualization plots with distance evolution for all trials."""
    
#     # Create figure with 1 row and 3 columns (since only 3 plots are used)
#     fig, axes = plt.subplots(1, 3, figsize=(20, 6))
#     fig.suptitle('Predator-Prey Behavior Analysis - All Trials', fontsize=16, fontweight='bold')
    
#     # 1. Bar chart of mean behavior counts
#     behaviors = list(stats.keys())
#     means = [stats[b]['mean'] for b in behaviors]
#     stds = [stats[b]['std'] for b in behaviors]
    
#     ax1 = axes[0]
#     bars = ax1.bar(behaviors, means, yerr=stds, capsize=5, color=['skyblue', 'lightcoral', 'lightgreen'])
#     ax1.set_title('Mean Behavior Counts per Trial')
#     ax1.set_ylabel('Count')
#     ax1.grid(axis='y', alpha=0.3)
    
#     # Add value labels on bars
#     for i, (bar, mean, std) in enumerate(zip(bars, means, stds)):
#         ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + std + 0.1,
#                 f'{mean:.1f}', ha='center', va='bottom')
    
#     # 2. Distance Evolution for ALL Trials - Individual Lines
#     ax2 = axes[1]
#     if all_results:
#         colors = plt.cm.tab10(np.linspace(0, 1, len(all_results)))
        
#         for i, result in enumerate(all_results):
#             distances = result['distances']
#             time_steps = np.arange(len(distances)) / DEFAULT_CONTROL_FREQ_HZ
#             ax2.plot(time_steps, distances, alpha=0.6, linewidth=1, 
#                     color=colors[i], label=f'Trial {i+1}')
        
#         # Add threshold lines
#         ax2.axhline(y=FOLLOW_DISTANCE_THRESHOLD, color='orange', linestyle='--', linewidth=2,
#                    label=f'Follow threshold ({FOLLOW_DISTANCE_THRESHOLD}m)')
#         ax2.axhline(y=ENGULF_DISTANCE_THRESHOLD, color='red', linestyle='--', linewidth=2,
#                    label=f'Engulf threshold ({ENGULF_DISTANCE_THRESHOLD}m)')
#         ax2.axhline(y=SEPARATION_DISTANCE_THRESHOLD, color='green', linestyle='--', linewidth=2,
#                    label=f'Separation threshold ({SEPARATION_DISTANCE_THRESHOLD}m)')
        
#         ax2.set_title('Distance Evolution - All Trials')
#         ax2.set_xlabel('Time (s)')
#         ax2.set_ylabel('Min Distance (m)')
#         ax2.grid(alpha=0.3)
        
#         # Only show threshold lines in legend
#         handles, labels = ax2.get_legend_handles_labels()
#         threshold_handles = handles[-3:]
#         threshold_labels = labels[-3:]
#         ax2.legend(threshold_handles, threshold_labels, loc='upper right')
    
#     # 3. Distance Evolution - Statistical Summary (Mean ± Std)
#     ax3 = axes[2]
#     if all_results:
#         min_length = min(len(result['distances']) for result in all_results)
        
#         aligned_distances = np.array([result['distances'][:min_length] for result in all_results])
        
#         mean_distances = np.mean(aligned_distances, axis=0)
#         std_distances = np.std(aligned_distances, axis=0)
#         time_steps = np.arange(min_length) / DEFAULT_CONTROL_FREQ_HZ
        
#         ax3.fill_between(time_steps, 
#                         mean_distances - std_distances,
#                         mean_distances + std_distances,
#                         alpha=0.3, color='blue', label='±1 Std Dev')
#         ax3.plot(time_steps, mean_distances, 'b-', linewidth=2, label='Mean Distance')
        
#         ax3.axhline(y=FOLLOW_DISTANCE_THRESHOLD, color='orange', linestyle='--', linewidth=2,
#                    label=f'Follow threshold ({FOLLOW_DISTANCE_THRESHOLD}m)')
#         ax3.axhline(y=ENGULF_DISTANCE_THRESHOLD, color='red', linestyle='--', linewidth=2,
#                    label=f'Engulf threshold ({ENGULF_DISTANCE_THRESHOLD}m)')
#         ax3.axhline(y=SEPARATION_DISTANCE_THRESHOLD, color='green', linestyle='--', linewidth=2,
#                    label=f'Separation threshold ({SEPARATION_DISTANCE_THRESHOLD}m)')
        
#         ax3.set_title('Distance Evolution - Mean ± Std')
#         ax3.set_xlabel('Time (s)')
#         ax3.set_ylabel('Min Distance (m)')
#         ax3.legend()
#         ax3.grid(alpha=0.3)
    
#     # Save the plot
#     dt_string = datetime.now().strftime("%d-%m-%Y_%H-%M")
#     save_dir = f"analysis_results_{dt_string}/"
#     if not os.path.exists(save_dir):
#         os.makedirs(save_dir)
    
#     plt.savefig(os.path.join(save_dir, 'behavior_analysis_all_trials.png'), dpi=300, bbox_inches='tight')
#     plt.show()
    
#     return save_dir


# def create_individual_distance_plots(all_results, save_dir):
#     """Create individual distance plots for each trial in a separate figure."""
    
#     # Calculate grid dimensions
#     n_trials = len(all_results)
#     n_cols = 3
#     n_rows = (n_trials + n_cols - 1) // n_cols
    
#     fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, 4 * n_rows))
#     fig.suptitle(f'Individual Distance Evolution - All {n_trials} Trials', fontsize=16)
    
#     # Flatten axes for easier indexing
#     if n_rows == 1:
#         axes = [axes] if n_cols == 1 else axes
#     else:
#         axes = axes.flatten()
    
#     for i, result in enumerate(all_results):
#         ax = axes[i] if n_trials > 1 else axes
        
#         distances = result['distances']
#         time_steps = np.arange(len(distances)) / DEFAULT_CONTROL_FREQ_HZ
        
#         # Plot distance evolution
#         ax.plot(time_steps, distances, 'b-', alpha=0.8, linewidth=1.5)
        
#         # Add threshold lines
#         ax.axhline(y=FOLLOW_DISTANCE_THRESHOLD, color='orange', linestyle='--', alpha=0.8,
#                   label=f'Follow ({FOLLOW_DISTANCE_THRESHOLD}m)')
#         ax.axhline(y=ENGULF_DISTANCE_THRESHOLD, color='red', linestyle='--', alpha=0.8,
#                   label=f'Engulf ({ENGULF_DISTANCE_THRESHOLD}m)')
#         ax.axhline(y=SEPARATION_DISTANCE_THRESHOLD, color='green', linestyle='--', alpha=0.8,
#                   label=f'Separation ({SEPARATION_DISTANCE_THRESHOLD}m)')
        
#         # Highlight behavior regions with background colors
#         ax.fill_between(time_steps, 0, ENGULF_DISTANCE_THRESHOLD, alpha=0.1, color='red', label='Engulf Zone')
#         ax.fill_between(time_steps, ENGULF_DISTANCE_THRESHOLD, FOLLOW_DISTANCE_THRESHOLD, 
#                        alpha=0.1, color='orange', label='Follow Zone')
#         ax.fill_between(time_steps, SEPARATION_DISTANCE_THRESHOLD, ax.get_ylim()[1], 
#                        alpha=0.1, color='green', label='Separation Zone')
        
#         ax.set_title(f'Trial {i+1}')
#         ax.set_xlabel('Time (s)')
#         ax.set_ylabel('Min Distance (m)')
#         ax.grid(alpha=0.3)
        
#         if i == 0:  # Only show legend for the first subplot
#             ax.legend(loc='upper right', fontsize=8)
        
#         # Add behavior statistics as text
#         behavior_counts = result['behavior_counts']
#         stats_text = f"F:{behavior_counts['following']} E:{behavior_counts['engulfing']} S:{behavior_counts['separating']}"
#         ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
#                verticalalignment='top', fontsize=8, 
#                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
#     # Hide unused subplots
#     for i in range(n_trials, len(axes)):
#         axes[i].set_visible(False)
    
#     plt.tight_layout()
#     plt.savefig(os.path.join(save_dir, 'individual_distance_plots.png'), dpi=300, bbox_inches='tight')
#     plt.show()

# def main():
#     """Main function to run multiple trials and analyze results."""
    
#     print("Starting Multi-Trial Predator-Prey Behavior Analysis")
#     print(f"Configuration:")
#     print(f"  Number of trials: {NUM_TRIALS}")
#     print(f"  Duration per trial: {DURATION_SEC} seconds")
#     print(f"  Number of predators: {NUM_DRONES}")
#     print(f"  Number of preys: {PREYS}")
#     print(f"  Follow threshold: {FOLLOW_DISTANCE_THRESHOLD}m")
#     print(f"  Engulf threshold: {ENGULF_DISTANCE_THRESHOLD}m")
#     print(f"  Separation threshold: {SEPARATION_DISTANCE_THRESHOLD}m")
#     print()
    
#     # Run trials
#     all_results = run_multiple_trials(NUM_TRIALS)
    
#     # Analyze results
#     stats, all_results = analyze_results(all_results)
    
#     # Create visualizations with all trials - using the corrected function
#     save_dir = create_visualizations_with_all_distances(stats, all_results)
    
#     # Create additional detailed distance analysis
#     create_detailed_distance_analysis(all_results, save_dir)
    
#     # Create individual distance plots
#     create_individual_distance_plots(all_results, save_dir)
    
#     # Save results
#     save_results(stats, all_results, save_dir)
    
#     print(f"\nAnalysis complete! Check the '{save_dir}' folder for results.")
#     print("Generated plots:")
#     print("  - behavior_analysis_all_trials.png (4 subplots with all distance evolutions)")
#     print("  - detailed_distance_analysis.png (4 additional detailed analysis plots)")
#     print("  - individual_distance_plots.png (separate plot for each trial)")

# if __name__ == "__main__":
#     main()

"""CrazyFlie software-in-the-loop control example with multi-trial analysis.

Setup
-----
Step 1: Clone pycffirmware from https://github.com/utiasDSL/pycffirmware
Step 2: Follow the install instructions for pycffirmware in its README

Example
-------
In terminal, run:
python gym_pybullet_drones/examples/cf_multi_trial.py

"""

import os
import sys

current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

import time
import argparse
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
import json

# # Configuration
NUM_TRIALS = 5  # Number of trials to run
BOUNDLESS = True
if BOUNDLESS:
    NS = 10
    PERC_NO_SENSOR = 0
else:
    NS = 5
    PERC_NO_SENSOR = 0

_3D = False
PLOT = False  # Disable plotting during background runs
GUI = False   # Disable GUI for background runs
REAL_TIME_SYNC = False  # DISABLED: No real-time synchronization
self_log = True

# Simulation parameters
DEFAULT_DRONES = DroneModel("cf2x")
DEFAULT_PHYSICS = Physics("pyb")
DEFAULT_SIMULATION_FREQ_HZ = 240
DEFAULT_CONTROL_FREQ_HZ = 48
DEFAULT_OUTPUT_FOLDER = 'results'
DURATION_SEC = 300  # Reduced for faster trials

NUM_DRONES = NS
PREYS = NS

# Behavior analysis parameters
FOLLOW_DISTANCE_THRESHOLD = 2.0     # Distance threshold for "following"
ENGULF_DISTANCE_THRESHOLD = 1.0     # Distance threshold for "engulfing"
SEPARATION_DISTANCE_THRESHOLD = 4.0  # Distance threshold for "separation"
MIN_BEHAVIOR_DURATION = 5           # Minimum frames to count as a behavior

class BehaviorAnalyzer:
    def __init__(self, num_predators, num_preys, ctrl_freq):
        self.num_predators = num_predators
        self.num_preys = num_preys
        self.ctrl_freq = ctrl_freq
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
        # Calculate average distances between predators and preys
        min_distances = []
        for i in range(self.num_predators):
            pred_pos = pred_positions[i]
            min_dist = float('inf')
            for j in range(self.num_preys):
                prey_pos = prey_positions[j]
                dist = np.linalg.norm(pred_pos - prey_pos)
                min_dist = min(min_dist, dist)
            min_distances.append(min_dist)
        
        avg_min_distance = np.mean(min_distances)
        self.distances.append(avg_min_distance)
        
        # Determine current behavior
        new_behavior = self._classify_behavior(avg_min_distance)
        
        # Check for behavior changes
        if new_behavior != self.current_behavior:
            # End previous behavior if it lasted long enough
            if (self.current_behavior is not None and 
                frame - self.behavior_start_frame >= MIN_BEHAVIOR_DURATION):
                self.behavior_counts[self.current_behavior] += 1
                self.behaviors.append({
                    'behavior': self.current_behavior,
                    'start_frame': self.behavior_start_frame,
                    'end_frame': frame,
                    'duration': frame - self.behavior_start_frame
                })
            
            # Start new behavior
            self.current_behavior = new_behavior
            self.behavior_start_frame = frame
    
    def _classify_behavior(self, distance):
        if distance <= ENGULF_DISTANCE_THRESHOLD:
            return 'engulfing'
        elif distance <= FOLLOW_DISTANCE_THRESHOLD:
            return 'following'
        elif distance >= SEPARATION_DISTANCE_THRESHOLD:
            return 'separating'
        else:
            return 'following'  # Default to following for intermediate distances
    
    def finalize_trial(self, total_frames):
        # Finalize the last behavior
        if (self.current_behavior is not None and 
            total_frames - self.behavior_start_frame >= MIN_BEHAVIOR_DURATION):
            self.behavior_counts[self.current_behavior] += 1
            self.behaviors.append({
                'behavior': self.current_behavior,
                'start_frame': self.behavior_start_frame,
                'end_frame': total_frames,
                'duration': total_frames - self.behavior_start_frame
            })
    
    def get_results(self):
        return {
            'behavior_counts': self.behavior_counts.copy(),
            'behaviors': self.behaviors.copy(),
            'distances': self.distances.copy()
        }

def run_single_trial(trial_num, duration_sec=DURATION_SEC, gui=False, plot=False):
    """Run a single trial and return behavior analysis results."""
    
    # Initialize positions for this trial
    drones_ids = list(range(NUM_DRONES))
    
    if _3D:
        min_distance = 3.
    else:
        if BOUNDLESS:
            min_distance = (np.sqrt(PREYS) * 1.7 * 0.3) + 3/1.5
        else:
            min_distance = 1.3

    if BOUNDLESS:
        init_center_x = 3 + np.random.uniform(-1, 1)  # Add randomness
        init_center_y = 3 + np.random.uniform(-1, 1)
        init_center_z = 3
        init_center_x_preys = init_center_x + np.random.choice([-1, 1]) * min_distance
        init_center_y_preys = init_center_y + np.random.choice([-1, 1]) * min_distance
        init_center_z_preys = init_center_z + 0
        spacing = 0.95
    else:
        init_center_x = 1
        init_center_y = 1
        init_center_z = 2
        init_center_x_preys = init_center_x + min_distance
        init_center_y_preys = init_center_y + min_distance
        init_center_z_preys = init_center_z + 0
        spacing = 0.2

    f_util = FlockingUtils(NUM_DRONES, PREYS, init_center_x, init_center_y, init_center_z,
                          spacing, init_center_x_preys, init_center_y_preys, init_center_z_preys,
                          drones_ids=drones_ids, perc_no_sensor=PERC_NO_SENSOR,
                          boundless=BOUNDLESS, _3D=_3D)

    # Initialize positions
    if _3D:
        pos_xs, pos_ys, pos_zs, pos_hxs, pos_hys, pos_hzs = f_util.initialize_positions3D(preds=True)
        pos_xs_preys, pos_ys_preys, pos_zs_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys = f_util.initialize_positions3D(preds=False)
    else:
        pos_xs, pos_ys, pos_zs, pos_h_xc, pos_h_yc, pos_h_zc = f_util.initialize_positions(preds=True)
        pos_xs_preys, pos_ys_preys, pos_zs_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys = f_util.initialize_positions(preds=False)

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

    # Create environment
    env = CtrlAviary_(drone_model=DEFAULT_DRONES,
                     num_drones=NUM_DRONES,
                     preys=PREYS,
                     initial_xyzs=INIT_XYZ,
                     initial_rpys=INIT_RPY,
                     initial_xyzs_preys=INIT_XYZ_PREYS,
                     initial_rpys_preys=INIT_RPY_PREYS,
                     physics=DEFAULT_PHYSICS,
                     record=False,
                     neighbourhood_radius=10,
                     pyb_freq=DEFAULT_SIMULATION_FREQ_HZ,
                     ctrl_freq=DEFAULT_CONTROL_FREQ_HZ,
                     gui=gui,
                     user_debug_gui=gui)

    # Controllers
    ctrl = [DSLPIDControl(drone_model=DEFAULT_DRONES) for i in range(NUM_DRONES)]
    ctrl_preys = [DSLPIDControl(drone_model=DEFAULT_DRONES) for i in range(PREYS)]

    # Initialize behavior analyzer
    analyzer = BehaviorAnalyzer(NUM_DRONES, PREYS, DEFAULT_CONTROL_FREQ_HZ)

    # START = time.time()  # REMOVED: No longer needed for timing sync
    action = np.zeros((NUM_DRONES, 4))
    action_preys = np.zeros((PREYS, 4))

    pos_x = np.zeros(NUM_DRONES)
    pos_y = np.zeros(NUM_DRONES)
    pos_z = np.zeros(NUM_DRONES)
    pos_x_preys = np.zeros(PREYS)
    pos_y_preys = np.zeros(PREYS)
    pos_z_preys = np.zeros(PREYS)

    total_frames = int(duration_sec * env.CTRL_FREQ)
    
    print(f"Running trial {trial_num + 1}/{NUM_TRIALS} (no real-time sync)...")

    for i in range(total_frames):
        obs, obs_preys, reward, reward_preys, done, done_preys, info, info_preys, _, _ = env.step(action, action_preys)
        
        # Get positions
        for j, k in zip(range(NUM_DRONES), range(PREYS)):
            states = env._getDroneStateVector(j)
            states_preys = env._getDroneStateVectorPreys(k)
            pos_x[j] = states[0]
            pos_y[j] = states[1]
            pos_z[j] = states[2]
            pos_x_preys[k] = states_preys[0]
            pos_y_preys[k] = states_preys[1]
            pos_z_preys[k] = states_preys[2]

        # Analyze behavior
        pred_positions = np.column_stack([pos_x, pos_y, pos_z])
        prey_positions = np.column_stack([pos_x_preys, pos_y_preys, pos_z_preys])
        analyzer.update(pred_positions, prey_positions, i)

        # Calculate forces and update positions
        f_util.calc_dij(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys)
        f_util.calc_ang_ij(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys)
        f_util.calc_grad_vals(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys, _3D)
        f_util.calc_p_forces()
        f_util.calc_p_forcesADM()
        f_util.calc_repulsion_predator_forces(pos_x, pos_y, pos_z, pos_x_preys, pos_y_preys, pos_z_preys)

        u, u_preys = f_util.calc_u_w()
        pos_hxs, pos_hys, pos_hzs, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys = f_util.get_heading()
        
        if not BOUNDLESS:
            f_util.calc_boun_rep(pos_x, pos_y, pos_z, pos_hxs, pos_hys, pos_hzs, preds=True)
            f_util.calc_boun_rep(pos_x_preys, pos_y_preys, pos_z_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys, preds=False)
        
        f_util.update_heading()

        # Control actions
        if _3D:
            for j, k in zip(range(NUM_DRONES), range(PREYS)):
                vel_cmd = np.array([u[j]*np.cos(pos_hxs[j]), u[j]*np.cos(pos_hys[j]), u[j]*np.cos(pos_hzs[j])])
                pos_cmd = np.array([pos_x[j], pos_y[j], pos_z[j]])
                action[j], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                state=obs[j], target_pos=pos_cmd,
                                                                target_vel=vel_cmd, target_rpy=np.array([0, 0, 0]))
                
                vel_cmd_preys = np.array([u_preys[k]*np.cos(pos_h_xc_preys[k]), u_preys[k]*np.cos(pos_h_yc_preys[k]), u_preys[k]*np.cos(pos_h_zc_preys[k])])
                pos_cmd_preys = np.array([pos_x_preys[k], pos_y_preys[k], pos_z_preys[k]])
                action_preys[k], _, _ = ctrl_preys[k].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                            state=obs_preys[k], target_pos=pos_cmd_preys,
                                                                            target_vel=vel_cmd_preys, target_rpy=np.array([0, 0, 0]))
        else:
            for j, k in zip(range(NUM_DRONES), range(PREYS)):
                vel_cmd = np.array([u[j] * np.cos(pos_hxs[j]), u[j] * np.cos(pos_hys[j]), 0])
                pos_cmd = np.array([pos_x[j], pos_y[j], pos_z[j]])
                action[j], _, _ = ctrl[j].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                state=obs[j], target_pos=pos_cmd,
                                                                target_vel=vel_cmd, target_rpy=np.array([0, 0, 0]))
                
                vel_cmd_preys = np.array([u_preys[k] * np.cos(pos_h_xc_preys[k]), u_preys[k] * np.cos(pos_h_yc_preys[k]), 0])
                pos_cmd_preys = np.array([pos_x_preys[k], pos_y_preys[k], pos_z_preys[k]])
                action_preys[k], _, _ = ctrl_preys[k].computeControlFromState(control_timestep=env.CTRL_TIMESTEP,
                                                                            state=obs_preys[k], target_pos=pos_cmd_preys,
                                                                            target_vel=vel_cmd_preys, target_rpy=np.array([0, 0, 0]))

        # Render if GUI is enabled, but WITHOUT real-time synchronization
        if gui:
            env.render()
            # REMOVED: sync(i, START, env.CTRL_TIMESTEP)  # No real-time sync

    # Finalize analysis
    analyzer.finalize_trial(total_frames)
    
    # Close environment
    env.close()
    
    return analyzer.get_results()

def run_multiple_trials(num_trials=NUM_TRIALS):
    """Run multiple trials and collect results."""
    all_results = []
    
    start_time = time.time()
    
    for trial in range(num_trials):
        trial_results = run_single_trial(trial, duration_sec=DURATION_SEC, gui=False, plot=False)
        trial_results['trial'] = trial
        all_results.append(trial_results)
    
    end_time = time.time()
    print(f"\nCompleted {num_trials} trials in {end_time - start_time:.2f} seconds")
    
    return all_results

def analyze_results(all_results):
    """Analyze and summarize results across all trials."""
    
    # Collect behavior counts
    following_counts = [r['behavior_counts']['following'] for r in all_results]
    engulfing_counts = [r['behavior_counts']['engulfing'] for r in all_results]
    separating_counts = [r['behavior_counts']['separating'] for r in all_results]
    
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
    print("\n" + "="*50)
    print("BEHAVIOR ANALYSIS SUMMARY")
    print("="*50)
    print(f"Number of trials: {len(all_results)}")
    print(f"Duration per trial: {DURATION_SEC} seconds")
    print(f"Real-time sync: {'ENABLED' if REAL_TIME_SYNC else 'DISABLED'}")
    print()
    
    for behavior, data in stats.items():
        print(f"{behavior.upper()}:")
        print(f"  Total occurrences: {data['total']}")
        print(f"  Mean per trial: {data['mean']:.2f} ± {data['std']:.2f}")
        print(f"  Range: {data['min']} - {data['max']}")
        print()
    
    return stats, all_results

def create_visualizations_with_all_distances(stats, all_results):
    """Create visualization plots with distance evolution for all trials."""
    
    # Create figure with 1 row and 3 columns (since only 3 plots are used)
    fig, axes = plt.subplots(1, 3, figsize=(20, 6))
    fig.suptitle('Predator-Prey Behavior Analysis - All Trials (No Real-Time Sync)', fontsize=16, fontweight='bold')
    
    # 1. Bar chart of mean behavior counts
    behaviors = list(stats.keys())
    means = [stats[b]['mean'] for b in behaviors]
    stds = [stats[b]['std'] for b in behaviors]
    
    ax1 = axes[0]
    bars = ax1.bar(behaviors, means, yerr=stds, capsize=5, color=['skyblue', 'lightcoral', 'lightgreen'])
    ax1.set_title('Mean Behavior Counts per Trial')
    ax1.set_ylabel('Count')
    ax1.grid(axis='y', alpha=0.3)
    
    # Add value labels on bars
    for i, (bar, mean, std) in enumerate(zip(bars, means, stds)):
        ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + std + 0.1,
                f'{mean:.1f}', ha='center', va='bottom')
    
    # 2. Distance Evolution for ALL Trials - Individual Lines
    ax2 = axes[1]
    if all_results:
        colors = plt.cm.tab10(np.linspace(0, 1, len(all_results)))
        
        for i, result in enumerate(all_results):
            distances = result['distances']
            time_steps = np.arange(len(distances)) / DEFAULT_CONTROL_FREQ_HZ
            ax2.plot(time_steps, distances, alpha=0.6, linewidth=1, 
                    color=colors[i], label=f'Trial {i+1}')
        
        # Add threshold lines
        ax2.axhline(y=FOLLOW_DISTANCE_THRESHOLD, color='orange', linestyle='--', linewidth=2,
                   label=f'Follow threshold ({FOLLOW_DISTANCE_THRESHOLD}m)')
        ax2.axhline(y=ENGULF_DISTANCE_THRESHOLD, color='red', linestyle='--', linewidth=2,
                   label=f'Engulf threshold ({ENGULF_DISTANCE_THRESHOLD}m)')
        ax2.axhline(y=SEPARATION_DISTANCE_THRESHOLD, color='green', linestyle='--', linewidth=2,
                   label=f'Separation threshold ({SEPARATION_DISTANCE_THRESHOLD}m)')
        
        ax2.set_title('Distance Evolution - All Trials')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Min Distance (m)')
        ax2.grid(alpha=0.3)
        
        # Only show threshold lines in legend
        handles, labels = ax2.get_legend_handles_labels()
        threshold_handles = handles[-3:]
        threshold_labels = labels[-3:]
        ax2.legend(threshold_handles, threshold_labels, loc='upper right')
    
    # 3. Distance Evolution - Statistical Summary (Mean ± Std)
    ax3 = axes[2]
    if all_results:
        min_length = min(len(result['distances']) for result in all_results)
        
        aligned_distances = np.array([result['distances'][:min_length] for result in all_results])
        
        mean_distances = np.mean(aligned_distances, axis=0)
        std_distances = np.std(aligned_distances, axis=0)
        time_steps = np.arange(min_length) / DEFAULT_CONTROL_FREQ_HZ
        
        ax3.fill_between(time_steps, 
                        mean_distances - std_distances,
                        mean_distances + std_distances,
                        alpha=0.3, color='blue', label='±1 Std Dev')
        ax3.plot(time_steps, mean_distances, 'b-', linewidth=2, label='Mean Distance')
        
        ax3.axhline(y=FOLLOW_DISTANCE_THRESHOLD, color='orange', linestyle='--', linewidth=2,
                   label=f'Follow threshold ({FOLLOW_DISTANCE_THRESHOLD}m)')
        ax3.axhline(y=ENGULF_DISTANCE_THRESHOLD, color='red', linestyle='--', linewidth=2,
                   label=f'Engulf threshold ({ENGULF_DISTANCE_THRESHOLD}m)')
        ax3.axhline(y=SEPARATION_DISTANCE_THRESHOLD, color='green', linestyle='--', linewidth=2,
                   label=f'Separation threshold ({SEPARATION_DISTANCE_THRESHOLD}m)')
        
        ax3.set_title('Distance Evolution - Mean ± Std')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Min Distance (m)')
        ax3.legend()
        ax3.grid(alpha=0.3)
    
    # Save the plot
    dt_string = datetime.now().strftime("%d-%m-%Y_%H-%M")
    save_dir = f"analysis_results_{dt_string}/"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    plt.savefig(os.path.join(save_dir, 'behavior_analysis_all_trials.png'), dpi=300, bbox_inches='tight')
    plt.show()
    
    return save_dir

def create_detailed_distance_analysis(all_results, save_dir):
    """Create detailed distance analysis plots."""
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Detailed Distance Analysis - All Trials (No Real-Time Sync)', fontsize=16, fontweight='bold')
    
    # Collect all distances and align them
    if not all_results:
        return
    
    min_length = min(len(result['distances']) for result in all_results)
    aligned_distances = np.array([result['distances'][:min_length] for result in all_results])
    time_steps = np.arange(min_length) / DEFAULT_CONTROL_FREQ_HZ
    
    # 1. Histogram of distances
    ax1 = axes[0, 0]
    all_distances_flat = aligned_distances.flatten()
    ax1.hist(all_distances_flat, bins=50, alpha=0.7, edgecolor='black')
    ax1.axvline(x=ENGULF_DISTANCE_THRESHOLD, color='red', linestyle='--', linewidth=2,
               label=f'Engulf ({ENGULF_DISTANCE_THRESHOLD}m)')
    ax1.axvline(x=FOLLOW_DISTANCE_THRESHOLD, color='orange', linestyle='--', linewidth=2,
               label=f'Follow ({FOLLOW_DISTANCE_THRESHOLD}m)')
    ax1.axvline(x=SEPARATION_DISTANCE_THRESHOLD, color='green', linestyle='--', linewidth=2,
               label=f'Separation ({SEPARATION_DISTANCE_THRESHOLD}m)')
    ax1.set_title('Distance Distribution (All Trials)')
    ax1.set_xlabel('Distance (m)')
    ax1.set_ylabel('Frequency')
    ax1.legend()
    ax1.grid(alpha=0.3)
    
    # 2. Time in each behavior zone
    ax2 = axes[0, 1]
    engulf_time = np.sum(aligned_distances <= ENGULF_DISTANCE_THRESHOLD, axis=1)
    follow_time = np.sum((aligned_distances > ENGULF_DISTANCE_THRESHOLD) & 
                        (aligned_distances <= FOLLOW_DISTANCE_THRESHOLD), axis=1)
    separate_time = np.sum(aligned_distances >= SEPARATION_DISTANCE_THRESHOLD, axis=1)
    
    # Convert to percentages
    total_time = aligned_distances.shape[1]
    engulf_pct = (engulf_time / total_time) * 100
    follow_pct = (follow_time / total_time) * 100
    separate_pct = (separate_time / total_time) * 100
    
    trials = np.arange(len(all_results))
    width = 0.25
    
    ax2.bar(trials - width, engulf_pct, width, label='Engulfing', color='red', alpha=0.7)
    ax2.bar(trials, follow_pct, width, label='Following', color='orange', alpha=0.7)
    ax2.bar(trials + width, separate_pct, width, label='Separating', color='green', alpha=0.7)
    
    ax2.set_title('Time Spent in Each Behavior Zone')
    ax2.set_xlabel('Trial')
    ax2.set_ylabel('Percentage of Time (%)')
    ax2.set_xticks(trials)
    ax2.set_xticklabels([f'T{i+1}' for i in trials])
    ax2.legend()
    ax2.grid(axis='y', alpha=0.3)
    
    # 3. Variance over time
    ax3 = axes[1, 0]
    distance_std = np.std(aligned_distances, axis=0)
    ax3.plot(time_steps, distance_std, 'b-', linewidth=2)
    ax3.set_title('Distance Variance Over Time')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Distance Standard Deviation (m)')
    ax3.grid(alpha=0.3)
    
    # 4. Minimum and maximum distances over time
    ax4 = axes[1, 1]
    distance_min = np.min(aligned_distances, axis=0)
    distance_max = np.max(aligned_distances, axis=0)
    distance_mean = np.mean(aligned_distances, axis=0)
    
    ax4.fill_between(time_steps, distance_min, distance_max, alpha=0.3, color='blue', label='Min-Max Range')
    ax4.plot(time_steps, distance_mean, 'b-', linewidth=2, label='Mean')
    ax4.axhline(y=FOLLOW_DISTANCE_THRESHOLD, color='orange', linestyle='--', linewidth=2,
               label=f'Follow threshold ({FOLLOW_DISTANCE_THRESHOLD}m)')
    ax4.axhline(y=ENGULF_DISTANCE_THRESHOLD, color='red', linestyle='--', linewidth=2,
               label=f'Engulf threshold ({ENGULF_DISTANCE_THRESHOLD}m)')
    ax4.axhline(y=SEPARATION_DISTANCE_THRESHOLD, color='green', linestyle='--', linewidth=2,
               label=f'Separation threshold ({SEPARATION_DISTANCE_THRESHOLD}m)')
    
    ax4.set_title('Distance Range Over Time')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Distance (m)')
    ax4.legend()
    ax4.grid(alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, 'detailed_distance_analysis.png'), dpi=300, bbox_inches='tight')
    plt.show()

def create_individual_distance_plots(all_results, save_dir):
    """Create individual distance plots for each trial in a separate figure."""
    
    # Calculate grid dimensions
    n_trials = len(all_results)
    n_cols = 3
    n_rows = (n_trials + n_cols - 1) // n_cols
    
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, 4 * n_rows))
    fig.suptitle(f'Individual Distance Evolution - All {n_trials} Trials (No Real-Time Sync)', fontsize=16)
    
    # Flatten axes for easier indexing
    if n_rows == 1:
        axes = [axes] if n_cols == 1 else axes
    else:
        axes = axes.flatten()
    
    for i, result in enumerate(all_results):
        ax = axes[i] if n_trials > 1 else axes
        
        distances = result['distances']
        time_steps = np.arange(len(distances)) / DEFAULT_CONTROL_FREQ_HZ
        
        # Plot distance evolution
        ax.plot(time_steps, distances, 'b-', alpha=0.8, linewidth=1.5)
        
        # Add threshold lines
        ax.axhline(y=FOLLOW_DISTANCE_THRESHOLD, color='orange', linestyle='--', alpha=0.8,
                  label=f'Follow ({FOLLOW_DISTANCE_THRESHOLD}m)')
        ax.axhline(y=ENGULF_DISTANCE_THRESHOLD, color='red', linestyle='--', alpha=0.8,
                  label=f'Engulf ({ENGULF_DISTANCE_THRESHOLD}m)')
        ax.axhline(y=SEPARATION_DISTANCE_THRESHOLD, color='green', linestyle='--', alpha=0.8,
                  label=f'Separation ({SEPARATION_DISTANCE_THRESHOLD}m)')
        
        # Highlight behavior regions with background colors
        ax.fill_between(time_steps, 0, ENGULF_DISTANCE_THRESHOLD, alpha=0.1, color='red', label='Engulf Zone')
        ax.fill_between(time_steps, ENGULF_DISTANCE_THRESHOLD, FOLLOW_DISTANCE_THRESHOLD, 
                       alpha=0.1, color='orange', label='Follow Zone')
        ax.fill_between(time_steps, SEPARATION_DISTANCE_THRESHOLD, ax.get_ylim()[1], 
                       alpha=0.1, color='green', label='Separation Zone')
        
        ax.set_title(f'Trial {i+1}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Min Distance (m)')
        ax.grid(alpha=0.3)
        
        if i == 0:  # Only show legend for the first subplot
            ax.legend(loc='upper right', fontsize=8)
        
        # Add behavior statistics as text
        behavior_counts = result['behavior_counts']
        stats_text = f"F:{behavior_counts['following']} E:{behavior_counts['engulfing']} S:{behavior_counts['separating']}"
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
               verticalalignment='top', fontsize=8, 
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Hide unused subplots
    for i in range(n_trials, len(axes)):
        axes[i].set_visible(False)
    
    plt.tight_layout()
    plt.savefig(os.path.join(save_dir, 'individual_distance_plots.png'), dpi=300, bbox_inches='tight')
    plt.show()

def save_results(stats, all_results, save_dir):
    """Save analysis results to files."""
    
    # Save statistics as JSON
    stats_file = os.path.join(save_dir, 'behavior_statistics.json')
    with open(stats_file, 'w') as f:
        json.dump(stats, f, indent=2)
    
    # Save detailed results as JSON
    results_file = os.path.join(save_dir, 'detailed_results.json')
    with open(results_file, 'w') as f:
        json.dump(all_results, f, indent=2)
    
    # Save summary as CSV
    summary_data = []
    for i, result in enumerate(all_results):
        row = {
            'trial': i + 1,
            'following_count': result['behavior_counts']['following'],
            'engulfing_count': result['behavior_counts']['engulfing'],
            'separating_count': result['behavior_counts']['separating'],
            'total_behaviors': sum(result['behavior_counts'].values()),
            'avg_distance': np.mean(result['distances']),
            'min_distance': np.min(result['distances']),
            'max_distance': np.max(result['distances']),
            'std_distance': np.std(result['distances'])
        }
        summary_data.append(row)
    
    summary_df = pd.DataFrame(summary_data)
    summary_file = os.path.join(save_dir, 'trial_summary.csv')
    summary_df.to_csv(summary_file, index=False)
    
    print(f"\nResults saved to:")
    print(f"  - Statistics: {stats_file}")
    print(f"  - Detailed results: {results_file}")
    print(f"  - Summary CSV: {summary_file}")

def main():
    """Main function to run multiple trials and analyze results."""
    
    print("Starting Multi-Trial Predator-Prey Behavior Analysis")
    print(f"Configuration:")
    print(f"  Number of trials: {NUM_TRIALS}")
    print(f"  Duration per trial: {DURATION_SEC} seconds")
    print(f"  Number of predators: {NUM_DRONES}")
    print(f"  Number of preys: {PREYS}")
    print(f"  Follow threshold: {FOLLOW_DISTANCE_THRESHOLD}m")
    print(f"  Engulf threshold: {ENGULF_DISTANCE_THRESHOLD}m")
    print(f"  Separation threshold: {SEPARATION_DISTANCE_THRESHOLD}m")
    print(f"  Real-time sync: {'ENABLED' if REAL_TIME_SYNC else 'DISABLED'}")
    print()
    
    # Run trials
    all_results = run_multiple_trials(NUM_TRIALS)
    
    # Analyze results
    stats, all_results = analyze_results(all_results)
    
    # Create visualizations with all trials
    save_dir = create_visualizations_with_all_distances(stats, all_results)
    
    # Create additional detailed distance analysis
    create_detailed_distance_analysis(all_results, save_dir)
    
    # Create individual distance plots
    create_individual_distance_plots(all_results, save_dir)
    
    # Save results
    save_results(stats, all_results, save_dir)
    
    print(f"\nAnalysis complete! Check the '{save_dir}' folder for results.")
    print("Generated plots:")
    print("  - behavior_analysis_all_trials.png (3 subplots with all distance evolutions)")
    print("  - detailed_distance_analysis.png (4 additional detailed analysis plots)")
    print("  - individual_distance_plots.png (separate plot for each trial)")
    print("\nPerformance benefit: Simulation runs at maximum speed without real-time constraints!")

if __name__ == "__main__":
    main()