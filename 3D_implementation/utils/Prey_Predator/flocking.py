
# import numpy as np
# from swarmPlot import SwarmPlotter

# class FlockingUtils:
#     def __init__(self, 
#                  n_predators, 
#                  preys, 
#                  center_x, 
#                  center_y, 
#                  center_z, 
#                  spacing, 
#                  center_x_preys, 
#                  center_y_preys, 
#                  center_z_preys, 
#                  drones_ids, 
#                  perc_no_sensor, 
#                  boundless,
#                  _3D:bool=False):
#         self.boundless = boundless

#         self._3D = _3D # bool
#         self.n_agents = n_predators
#         self.n_preys = preys
#         self.center_x = center_x
#         self.center_y = center_y
#         self.center_z = center_z
        
#         self.center_x_preys = center_x_preys
#         self.center_y_preys = center_y_preys
#         self.center_z_preys = center_z_preys
#         self.spacing = spacing

#         self.drones_ids = drones_ids
#         self.perc_no_sensor = perc_no_sensor
#         self.boun_thresh = 0.5
#         self.Dr = 1.0
#         self.k_rep = 2.0
#         self.L0 = 0.5
#         self.boun_x = 7.0
#         self.boun_y = 5.0
#         self.boun_z = 5.0
#         self.id_to_index = {id: index for index, id in enumerate(self.drones_ids)}
#         self.no_sensor_predator_ids = []

#         # ============================================================
#         # SYMMETRICAL PARAMETERS - SAME FOR BOTH PREDATORS AND PREYS
#         # ============================================================
#         if not _3D:
#             if self.boundless:
#                 # === BOUNDLESS 2D MODE ===
#                 self.sensing_range = 3  # sensing range for preys
#                 self.sensor_range  = 3  # sensing range for predators
#                 self.Dp = 3           # Detection range for predators
#                 self.Dp_preys = 3     # Detection range for preys (SAME)
#                 self.sigma = 1.2/2**0.5          # Sigma for predators
#                 self.sigma_no_sensor = 3.0 #ignore this as it for for non sensing predator
#                 self.kappa = 1.5
#                 self.sigma_prey =  1.2/2**0.5     # Sigma for preys (SAME)
#             else:
#                 # === BOUNDED 2D MODE ===
#                 self.sensing_range = 2.0
#                 self.sensor_range  = 2.0
#                 self.Dp = 2.0
#                 self.Dp_preys = 2.0       # SAME
#                 self.sigma = 0.5
#                 self.sigma_no_sensor = 0.5
#                 self.kappa = 1.5
#                 self.sigma_prey = 0.5     # SAME
            
#             # Common 2D parameters
#             self.sigmas = self.sigma * np.ones(self.n_agents)
#             bound_alpha = 0.5
#             self.sigmas_b = bound_alpha * np.ones(self.n_agents)
            
#             self.sigmas_preys = self.sigma_prey * np.ones(self.n_preys)
#             self.sigmas_b_preys = bound_alpha * np.ones(self.n_preys)

#             self.epsilon = 12.0
#             self.alpha = 2          # Alpha for predators
#             self.alpha_preys = 2.0    # Alpha for preys (SAME)
#             self.beta = 2.0
#         else:
#             # === 3D MODE ===
#             self.Dp = 4.0
#             self.Dp_preys = 4.0           # SAME

#             self.sensing_range = 3.0
#             self.sensor_range  = 3.0

#             self.sigma = 1.4
#             self.sigma_no_sensor = 1.6
#             self.sigma_prey = 1.4         # SAME as predators
            
#             self.sigmas = self.sigma * np.ones(self.n_agents)
#             self.sigmas_b = 0.05 * np.ones(self.n_agents)

#             self.sigmas_preys = self.sigma_prey * np.ones(self.n_preys)
#             self.sigmas_b_preys = 0.05 * np.ones(self.n_preys)

#             self.epsilon = 12.0
#             self.alpha = 3.0          # Alpha for predators
#             self.alpha_preys = 3.0    # Alpha for preys (SAME)
#             self.beta = 2.0

#         # Common parameters for both 2D and 3D
#         self.k1 = 0.5
#         self.k2 = 0.1
#         self.umax_const = 0.20
#         self.wmax = 1.5708
#         self.h_alignment = False
#         self.dt = 0.05
#         self.noise_pos = 0.03
#         self.noise_h = np.pi / 90
#         self.rng = np.random.default_rng(1234)
#         self.mean_noise = 0.3

#         # Initialize arrays
#         self.d_ij = np.zeros([self.n_agents, self.n_agents])
#         self.pos_h_xc = np.zeros(self.n_agents)
#         self.pos_h_yc = np.zeros(self.n_agents)
#         self.pos_h_zc = np.zeros(self.n_agents)
#         self.ij_ang_x = np.zeros([self.n_agents, self.n_agents])
#         self.ij_ang_y = np.zeros([self.n_agents, self.n_agents])
#         self.ij_ang_z = np.zeros([self.n_agents, self.n_agents])
#         self.f_x = np.zeros(self.n_agents)
#         self.f_y = np.zeros(self.n_agents)
#         self.f_z = np.zeros(self.n_agents)
#         self.fa_x = np.zeros(self.n_agents)
#         self.fa_y = np.zeros(self.n_agents)
#         self.fa_z = np.zeros(self.n_agents)
#         self.u = np.zeros(self.n_agents)
#         self.w = np.zeros(self.n_agents)

#         self.d_ij_from_preys = np.zeros([self.n_agents, self.n_preys])
#         self.d_ij_from_predators = np.zeros([self.n_preys, self.n_agents])

#         self.d_ij_preys = np.zeros([self.n_preys, self.n_preys])
#         self.pos_h_xc_preys = np.zeros(self.n_preys)
#         self.pos_h_yc_preys = np.zeros(self.n_preys)
#         self.pos_h_zc_preys = np.zeros(self.n_preys)
#         self.ij_ang_x_preys = np.zeros([self.n_preys, self.n_preys])
#         self.ij_ang_y_preys = np.zeros([self.n_preys, self.n_preys])
#         self.ij_ang_z_preys = np.zeros([self.n_preys, self.n_preys])
#         self.f_x_preys = np.zeros(self.n_preys)
#         self.f_y_preys = np.zeros(self.n_preys)
#         self.f_z_preys = np.zeros(self.n_preys)
#         self.fa_x_preys = np.zeros(self.n_preys)
#         self.fa_y_preys = np.zeros(self.n_preys)
#         self.fa_z_preys = np.zeros(self.n_preys)
#         self.u_preys = np.zeros(self.n_preys)
#         self.w_preys = np.zeros(self.n_preys)

#         map_x, map_y, map_z = [np.linspace(-1, 1, 150) for _ in range(3)]
#         X, Y, Z = np.meshgrid(map_x, map_y, map_z)
#         self.map_3d = 255 * np.exp(-(X ** 2 + Y ** 2 + Z ** 2) / (2 * self.sigma ** 2))
#         self.grad_const_x, self.grad_const_y, self.grad_const_z = [150 / boun for boun in (self.boun_x, self.boun_y, self.boun_z)]
        
#         self.plotter = SwarmPlotter(self.n_agents, 
#                                     self.n_preys, 
#                                     self.boun_x, 
#                                     self.boun_y, 
#                                     self.boun_z, 
#                                     no_sensor_percentage=self.perc_no_sensor, 
#                                     boundless=self.boundless)

#     def _calculate_symmetrical_dimensions(self, num_agents):
#         """Helper to find the best (x, y) dimensions for a perfect or near-perfect grid."""
#         # 1. Prioritize a perfect, non-square fit
#         for x in range(int(np.ceil(np.sqrt(num_agents))), 0, -1):
#             if num_agents % x == 0:
#                 y = num_agents // x
#                 return (x, y) 

#         # 2. If no perfect rectangle, fall back to the most square-like
#         square_root = round(num_agents ** 0.5)
#         best_diff = float('inf')
#         dimensions = (0, 0)
#         for x in range(square_root, 0, -1):
#             y = int(np.ceil(num_agents / x))
#             total_agents = x * y
#             diff = abs(x - y)
#             if diff < best_diff and total_agents >= num_agents:
#                 best_diff = diff
#                 dimensions = (x, y)
#         return dimensions

#     def initialize_positions(self, preds: bool = True):
#         """Place agents in a 2D grid around an initialization point with specified spacing and noise."""
#         if preds:
#             num_agents = self.n_agents
#             init_pos = (self.center_x, self.center_y, self.center_z)
#         else:  # preys
#             num_agents = self.n_preys
#             init_pos = (self.center_x_preys, self.center_y_preys, self.center_z_preys)

#         spacing = self.spacing
#         mean_noise = self.mean_noise

#         # Calculate dimensions ONLY ONCE based on the largest group size
#         max_agents = max(self.n_agents, self.n_preys)
#         dimensions = self._calculate_symmetrical_dimensions(max_agents)

#         # Generate 2D grid positions
#         grid_positions = np.mgrid[0:dimensions[0], 0:dimensions[1]].reshape(2, -1).T
#         grid_positions = grid_positions * spacing

#         # Center the grid around the init_pos (in x and y only)
#         offset = np.array(init_pos[:2]) - (np.array(dimensions) * spacing / 2)
#         grid_positions += offset

#         # Apply noise
#         noise = np.random.normal(loc=mean_noise, scale=mean_noise / 3, size=grid_positions.shape)
#         grid_positions += noise

#         # Set z positions to the z component of init_pos
#         z_positions = np.full(grid_positions.shape[0], init_pos[2])

#         if preds:
#             theta = np.random.uniform(0, 2 * np.pi, self.n_agents)
#             phi = np.random.uniform(0, np.pi, self.n_agents)

#             self.pos_h_xc = np.sin(phi) * np.cos(theta)
#             self.pos_h_yc = np.sin(phi) * np.sin(theta)
#             self.pos_h_zc = np.cos(phi)

#             self.init_pos_preds = grid_positions 
#             return (grid_positions[:num_agents, 0], grid_positions[:num_agents, 1], z_positions[:num_agents],
#                     self.pos_h_xc, self.pos_h_yc, self.pos_h_zc)
#         else:  # preys
#             theta = np.random.uniform(0, 2 * np.pi, self.n_preys)
#             phi = np.random.uniform(0, np.pi, self.n_preys)

#             self.pos_h_xc_preys = np.sin(phi) * np.cos(theta)
#             self.pos_h_yc_preys = np.sin(phi) * np.sin(theta)
#             self.pos_h_zc_preys = np.cos(phi)

#             self.init_pos_preys = grid_positions
            
#             # Calculate no-sensor predators
#             self.no_sensor_predator_ids = self.no_sensor_predators(
#                 self.perc_no_sensor, 
#                 self.drones_ids, 
#                 self.init_pos_preds, 
#                 self.init_pos_preys
#             )
#             self.plotter.no_sensor_preds(self.no_sensor_predator_ids)

#             return (grid_positions[:num_agents, 0], grid_positions[:num_agents, 1], z_positions[:num_agents],
#                     self.pos_h_xc_preys, self.pos_h_yc_preys, self.pos_h_zc_preys)

#     def no_sensor_predators(self, no_sensor_percentage: float, Ids: list, init_pos_preds: np.array, init_pos_preys: np.array):
#         """Calculate which predators should have no sensors based on distance to preys."""
#         distances = [min(np.linalg.norm(predator - prey) for prey in init_pos_preys) for predator in init_pos_preds]
#         id_distance_pairs = list(zip(Ids, distances))
#         id_distance_pairs.sort(key=lambda x: x[1])
#         sorted_ids = [id for id, distance in id_distance_pairs]
#         sorted_ids.reverse()
#         no_sensor_agents = int(no_sensor_percentage * self.n_agents)
#         no_sensor_predators = sorted_ids[:no_sensor_agents]
#         return no_sensor_predators

#     def calculate_rotated_vector_batch(self, X1, Y1, Z1, X2, Y2, Z2, wdt):
#         """Rodrigues' rotation formula for batch vector rotation."""
#         vector1 = np.stack([X1, Y1, Z1], axis=-1)
#         vector2 = np.stack([X2, Y2, Z2], axis=-1)

#         original_magnitude = np.linalg.norm(vector1, axis=1, keepdims=True)
#         vector2_magnitude = np.linalg.norm(vector2, axis=1, keepdims=True)

#         vector2_normalized_scaled = vector2 * (original_magnitude / vector2_magnitude)

#         normal_vector = np.cross(vector1, vector2_normalized_scaled)
#         normal_magnitude = np.linalg.norm(normal_vector, axis=1, keepdims=True)
#         normal_vector /= np.where(normal_magnitude > 0, normal_magnitude, 1)

#         k_cross_vector1 = np.cross(normal_vector, vector1)
#         cos_theta = np.cos(wdt)[:, np.newaxis]
#         sin_theta = np.sin(wdt)[:, np.newaxis]
#         one_minus_cos_theta = (1 - cos_theta)

#         dot_product = np.sum(normal_vector * vector1, axis=1, keepdims=True)
#         v_rot = vector1 * cos_theta + k_cross_vector1 * sin_theta + normal_vector * dot_product * one_minus_cos_theta

#         return v_rot.T

#     def calculate_av_heading(self, x_components, y_components, z_components):
#         """Calculate average heading direction from heading components."""
#         normalized_vectors = []
#         for x, y, z in zip(x_components, y_components, z_components):
#             vec = np.array([x, y, z])
#             norm = np.linalg.norm(vec)
#             if norm != 0:
#                 normalized_vectors.append(vec / norm)
#             else:
#                 normalized_vectors.append(vec)

#         sum_of_normalized_vectors = np.sum(normalized_vectors, axis=0)
#         unit_vector_average_direction = sum_of_normalized_vectors / np.linalg.norm(sum_of_normalized_vectors)
#         return unit_vector_average_direction

#     def detect_bounds(self, pos_x, pos_y, pos_z):
#         """Detect if agents are near boundaries."""
#         result_x = np.zeros_like(pos_x)
#         result_y = np.zeros_like(pos_y)
#         result_z = np.zeros_like(pos_z)

#         result_x[pos_x < self.boun_thresh] = 1
#         result_x[pos_x > self.boun_x - self.boun_thresh] = -1

#         result_y[pos_y < self.boun_thresh] = 1
#         result_y[pos_y > self.boun_y - self.boun_thresh] = -1

#         result_z[pos_z < self.boun_thresh] = 1
#         result_z[pos_z > self.boun_z - self.boun_thresh] = -1

#         return result_x, result_y, result_z

#     def calc_dij(self, pos_xs, pos_ys, pos_zs, pos_x_preys, pos_y_preys, pos_z_preys):
#         """Calculate pairwise distances between agents."""
#         self.d_ij = np.hypot(np.hypot(pos_xs[:, None] - pos_xs, pos_ys[:, None] - pos_ys), pos_zs[:, None] - pos_zs)
#         self.d_ij[(self.d_ij > self.Dp) | (self.d_ij == 0)] = np.inf
#         self.d_ij_noise = self.d_ij + self.rng.uniform(-self.noise_pos, self.noise_pos, (self.n_agents, self.n_agents)) * self.dt

#         self.d_ij_preys = np.hypot(np.hypot(pos_x_preys[:, None] - pos_x_preys, pos_y_preys[:, None] - pos_y_preys), pos_z_preys[:, None] - pos_z_preys)
#         self.d_ij_preys[(self.d_ij_preys > self.Dp_preys) | (self.d_ij_preys == 0)] = np.inf
#         self.d_ij_noise_preys = self.d_ij_preys + self.rng.uniform(-self.noise_pos, self.noise_pos, (self.n_preys, self.n_preys)) * self.dt

#     def calc_ang_ij(self, pos_xs, pos_ys, pos_zs, pos_x_preys, pos_y_preys, pos_z_preys):
#         """Calculate angular relationships between agents."""
#         self.ij_ang_x = np.arccos((pos_xs - pos_xs[:, None]) / self.d_ij)
#         self.ij_ang_y = np.arccos((pos_ys - pos_ys[:, None]) / self.d_ij)
#         self.ij_ang_z = np.arccos((pos_zs - pos_zs[:, None]) / self.d_ij)

#         self.ij_ang_x_preys = np.arccos((pos_x_preys - pos_x_preys[:, None]) / self.d_ij_preys)
#         self.ij_ang_y_preys = np.arccos((pos_y_preys - pos_y_preys[:, None]) / self.d_ij_preys)
#         self.ij_ang_z_preys = np.arccos((pos_z_preys - pos_z_preys[:, None]) / self.d_ij_preys)

#     def calc_grad_vals(self, pos_xs, pos_ys, pos_zs, pos_x_preys, pos_y_preys, pos_z_preys, _3D):
#         """Calculate gradient values for adaptive distance modulation."""
#         self.d_ij_from_preys = np.hypot(np.hypot(pos_xs - pos_x_preys[:, None], pos_ys - pos_y_preys[:, None]), pos_zs - pos_z_preys[:, None])
#         self.d_ij_from_preys[(self.d_ij_from_preys > self.sensor_range) | (self.d_ij_from_preys == 0)] = np.inf

#         self.d_ij_from_preys_noise = self.d_ij_from_preys + self.rng.uniform(-self.noise_pos, self.noise_pos, (self.n_preys, self.n_agents)) * self.dt

#         # Calculate the average distance for each predator
#         avg_distances = np.zeros(self.n_agents)
#         for i in range(self.n_agents):
#             within_range_distances = self.d_ij_from_preys_noise[:, i][self.d_ij_from_preys_noise[:, i] != np.inf]
#             if within_range_distances.size > 0:
#                 avg_distances[i] = np.mean(within_range_distances)
#             else:
#                 avg_distances[i] = np.inf

#         # Convert IDs to numerical indices for indexing
#         no_sensor_indices = [self.id_to_index[id] for id in self.no_sensor_predator_ids]        
#         avg_distances[no_sensor_indices] = -np.inf

#         # # Update sigma values based on average distances
#         # if _3D:
#         #     self.sigmas = np.where(avg_distances == -np.inf, self.sigma_no_sensor, (self.sigma + 4.5 * (1 / avg_distances)))
#         # else:
#         #     if self.boundless:
#         #         self.sigmas = np.where(avg_distances == -np.inf, self.sigma_no_sensor, (self.sigma + 1.5 * (1 / avg_distances)))
#         #     else:
#         #         self.sigmas = np.where(avg_distances == -np.inf, self.sigma_no_sensor, (self.sigma + 0.3 * (1 / avg_distances)))

#     def calc_repulsion_predator_forces(self, pos_xs, pos_ys, pos_zs, pos_x_preys, pos_y_preys, pos_z_preys):
#         """Calculate repulsion forces on preys from predators."""
#         positions_preys = np.column_stack((pos_x_preys, pos_y_preys, pos_z_preys))
#         positions_predators = np.column_stack((pos_xs, pos_ys, pos_zs))
#         N = len(pos_x_preys)

#         # Calculate the distance matrix
#         distance_matrix = np.linalg.norm(positions_preys[:, np.newaxis] - positions_predators[np.newaxis], axis=-1)
#         mask = distance_matrix <= self.sensing_range

#         # Add noise to the distance matrix
#         distance_matrix_noise = distance_matrix + self.rng.uniform(-self.noise_pos, self.noise_pos, distance_matrix.shape) * self.dt
#         distance_matrix_noise[(distance_matrix_noise > self.sensing_range) | (distance_matrix_noise == 0)] = np.inf

#         # Initialize repulsion vector
#         repulsion_vector = np.zeros((N, 3))

#         for i in range(N):
#             within_range_indices = np.where(mask[i])[0]

#             if within_range_indices.size > 0:
#                 # Calculate the average position of predators within range
#                 average_preds = np.mean(positions_predators[within_range_indices], axis=0)

#                 # Calculate the direction vector from prey to average position of nearby predators
#                 direction_x = positions_preys[i, 0] - average_preds[0]
#                 direction_y = positions_preys[i, 1] - average_preds[1]
#                 direction_z = positions_preys[i, 2] - average_preds[2]
#                 distance = np.linalg.norm([direction_x, direction_y, direction_z])
#                 kappa = 4 / (distance + 1)

#                 # Update the repulsion vector
#                 repulsion_vector[i, 0] = kappa * direction_x
#                 repulsion_vector[i, 1] = kappa * direction_y
#                 repulsion_vector[i, 2] = kappa * direction_z

#         # Update forces on preys
#         self.f_x_preys += repulsion_vector[:, 0]
#         self.f_y_preys += repulsion_vector[:, 1]
#         self.f_z_preys += repulsion_vector[:, 2]

#     def calc_p_forcesADM(self):
#         """Calculate ADM forces for PREDATORS."""
#         forces = -self.epsilon * (self.sigmas[:, np.newaxis] / self.d_ij_noise - 
#                                   np.sqrt(self.sigmas[:, np.newaxis] / self.d_ij_noise))

#         cos_ij_ang_x = np.cos(self.ij_ang_x)
#         cos_ij_ang_y = np.cos(self.ij_ang_y)
#         cos_ij_ang_z = np.cos(self.ij_ang_z)

#         self.f_x = self.alpha * np.sum(forces * cos_ij_ang_x, axis=1)
#         self.f_x = np.where(self.f_x == 0, 0.00001, self.f_x)

#         self.f_y = self.alpha * np.sum(forces * cos_ij_ang_y, axis=1)
#         self.f_y = np.where(self.f_y == 0, 0.00001, self.f_y)

#         self.f_z = self.alpha * np.sum(forces * cos_ij_ang_z, axis=1)
#         self.f_z = np.where(self.f_z == 0, 0.00001, self.f_z)

#     def calc_p_forces(self):
#         """Calculate ADM forces for PREYS (same formula as predators for symmetry)."""
#         forces_preys = -self.epsilon * (self.sigmas_preys[:, np.newaxis] / self.d_ij_noise_preys - 
#                                         np.sqrt(self.sigmas_preys[:, np.newaxis] / self.d_ij_noise_preys))

#         cos_ij_ang_x = np.cos(self.ij_ang_x_preys)
#         cos_ij_ang_y = np.cos(self.ij_ang_y_preys)
#         cos_ij_ang_z = np.cos(self.ij_ang_z_preys)

#         self.f_x_preys = self.alpha_preys * np.sum(forces_preys * cos_ij_ang_x, axis=1)
#         self.f_x_preys = np.where(self.f_x_preys == 0, 0.00001, self.f_x_preys)

#         self.f_y_preys = self.alpha_preys * np.sum(forces_preys * cos_ij_ang_y, axis=1)
#         self.f_y_preys = np.where(self.f_y_preys == 0, 0.00001, self.f_y_preys)

#         self.f_z_preys = self.alpha_preys * np.sum(forces_preys * cos_ij_ang_z, axis=1)
#         self.f_z_preys = np.where(self.f_z_preys == 0, 0.00001, self.f_z_preys)

#     def calc_alignment_forces(self):
#         """Calculate alignment forces for both predators and preys."""
#         av_heading = self.calculate_av_heading(self.pos_h_xc, self.pos_h_yc, self.pos_h_zc)
#         av_heading_preys = self.calculate_av_heading(self.pos_h_xc_preys, self.pos_h_yc_preys, self.pos_h_zc_preys)

#         self.fa_x = int(self.h_alignment) * self.beta * av_heading[0]
#         self.fa_y = int(self.h_alignment) * self.beta * av_heading[1]
#         self.fa_z = int(self.h_alignment) * self.beta * av_heading[2]

#         self.fa_x_preys = int(self.h_alignment) * self.beta * av_heading_preys[0]
#         self.fa_y_preys = int(self.h_alignment) * self.beta * av_heading_preys[1]
#         self.fa_z_preys = int(self.h_alignment) * self.beta * av_heading_preys[2]

#     def rotate_vector(self, vector, angle):
#         """Rotates a 2D vector by a given angle."""
#         rotation_matrix = np.array([
#             [np.cos(angle), -np.sin(angle)],
#             [np.sin(angle), np.cos(angle)]
#         ])
#         return np.dot(rotation_matrix, vector)

#     def calc_boun_rep(self, pos_x_preys, pos_y_preys, pos_z_preys, pos_h_xc_preys, pos_h_yc_preys, pos_h_zc_preys, preds: bool):
#         """Calculate boundary repulsion forces."""
#         boundaries = [self.boun_x, self.boun_y]
#         boundaryX, boundaryY = boundaries
#         position_ix = pos_x_preys
#         position_iy = pos_y_preys

#         boundary_x = np.array([0, boundaryX])
#         boundary_y = np.array([0, boundaryY])

#         # Compute distances to the four boundaries
#         dists_x = np.minimum(np.abs(position_ix[:, np.newaxis] - boundary_x[0]),
#                             np.abs(position_ix[:, np.newaxis] - boundary_x[1]))
#         dists_y = np.minimum(np.abs(position_iy[:, np.newaxis] - boundary_y[0]),
#                             np.abs(position_iy[:, np.newaxis] - boundary_y[1]))
        
#         dists = np.stack((dists_x, dists_y), axis=-1)

#         r_vector_preys = np.zeros((len(pos_x_preys), 2))

#         # Compute r vector for each prey and add it to the r_vector matrix
#         for i, (pos_x, pos_y) in enumerate(zip(position_ix, position_iy)):
#             for j, dist in enumerate(dists[i].flatten()):
#                 if dist < self.Dr:
#                     # Compute angle based on which boundary we're closest to
#                     Lb = dist
#                     if Lb == 0.5 or Lb == 0:
#                         Lb += 1.5

#                     # Determine angle based on the closest boundary
#                     if j == 0:  # X boundaries
#                         if pos_x < boundary_x[0] + self.Dr:  # Close to left boundary
#                             angle = np.pi  # 180 degrees
#                         else:  # Close to right boundary
#                             angle = 0  # 0 degrees
#                     else:  # Y boundaries
#                         if pos_y < boundary_y[0] + self.Dr:  # Close to bottom boundary
#                             angle = -np.pi / 2  # -90 degrees
#                         else:  # Close to top boundary
#                             angle = np.pi / 2  # 90 degrees

#                     pb_i = np.array([np.cos(angle), np.sin(angle)])

#                     # Adjust repulsion vector calculation
#                     if j < 2:  # Vertical boundaries
#                         rb_i = -self.k_rep * ((1.0 / Lb) - (1.0 / self.L0)) * (pb_i / (Lb**3))
#                     else:  # Horizontal boundaries
#                         rb_i = -self.k_rep * ((1.0 / Lb) - (1.0 / self.L0)) * (pb_i / (Lb**3))
                    
#                     # Rotate the repulsion vector
#                     angle_rotation = np.pi / 4
#                     rb_i = self.rotate_vector(rb_i, angle_rotation)

#                     r_vector_preys[i] += rb_i

#         r_vector_x = r_vector_preys[:, 0]
#         r_vector_y = r_vector_preys[:, 1]

#         if preds:
#             self.f_x += self.fa_x + r_vector_x
#             self.f_y += self.fa_y + r_vector_y
#         else:
#             self.f_x_preys += self.fa_x_preys + r_vector_x
#             self.f_y_preys += self.fa_y_preys + r_vector_y

#     def calc_u_w(self):
#         """Calculate linear and angular velocities for both predators and preys."""
#         f_mag = np.sqrt(np.square(self.f_x) + np.square(self.f_y) + np.square(self.f_z))
#         f_mag_preys = np.sqrt(np.square(self.f_x_preys) + np.square(self.f_y_preys) + np.square(self.f_z_preys))

#         f_mag = np.where(f_mag == 0, 0.00001, f_mag)
#         f_mag_preys = np.where(f_mag_preys == 0, 0.00001, f_mag_preys)

#         dot_f_h = self.f_x * self.pos_h_xc + self.f_y * self.pos_h_yc
#         dot_f_h_preys = self.f_x_preys * self.pos_h_xc_preys + self.f_y_preys * self.pos_h_yc_preys

#         cos_dot_f_h = np.clip(dot_f_h / f_mag, -1.0, 1.0)
#         cos_dot_f_h_preys = np.clip(dot_f_h_preys / f_mag_preys, -1.0, 1.0)

#         ang_f_h = np.arccos(cos_dot_f_h)
#         ang_f_h_preys = np.arccos(cos_dot_f_h_preys)
#         ang_f_h += self.rng.uniform(-self.noise_h, self.noise_h, self.n_agents) * self.dt

#         self.u = self.k1 * f_mag * np.cos(ang_f_h) + 0.05
#         self.w = self.k2 * f_mag * np.sin(ang_f_h)

#         self.u_preys = self.k1 * f_mag_preys * np.cos(ang_f_h_preys) + 0.05
#         self.w_preys = self.k2 * f_mag_preys * np.sin(ang_f_h_preys)

#         self.u = np.clip(self.u, 0, self.umax_const)
#         self.w = np.clip(self.w, -self.wmax, self.wmax)

#         self.u_preys = np.clip(self.u_preys, 0, self.umax_const)
#         self.w_preys = np.clip(self.w_preys, -self.wmax, self.wmax)

#         return self.u, self.u_preys

#     def get_heading(self):
#         """Get heading angles for both predators and preys."""
#         pos_h_m = np.sqrt(np.square(self.pos_h_xc) + np.square(self.pos_h_yc) + np.square(self.pos_h_zc))
#         pos_h_m_preys = np.sqrt(np.square(self.pos_h_xc_preys) + np.square(self.pos_h_yc_preys) + np.square(self.pos_h_zc_preys))

#         pos_hxs = np.arccos(self.pos_h_xc / pos_h_m)
#         pos_hys = np.arccos(self.pos_h_yc / pos_h_m)
#         pos_hzs = np.arccos(self.pos_h_zc / pos_h_m)

#         pos_hxs_preys = np.arccos(self.pos_h_xc_preys / pos_h_m_preys)
#         pos_hys_preys = np.arccos(self.pos_h_yc_preys / pos_h_m_preys)
#         pos_hzs_preys = np.arccos(self.pos_h_zc_preys / pos_h_m_preys)

#         return pos_hxs, pos_hys, pos_hzs, pos_hxs_preys, pos_hys_preys, pos_hzs_preys

#     def update_heading(self):
#         """Update heading vectors for both predators and preys."""
#         v_rot = self.calculate_rotated_vector_batch(
#             self.pos_h_xc, self.pos_h_yc, self.pos_h_zc, self.f_x, self.f_y, self.f_z, self.w * self.dt)
#         v_rot_preys = self.calculate_rotated_vector_batch(
#             self.pos_h_xc_preys, self.pos_h_yc_preys, self.pos_h_zc_preys, self.f_x_preys, self.f_y_preys, self.f_z_preys, self.w_preys * self.dt)

#         self.pos_h_xc = v_rot[0, :]
#         self.pos_h_yc = v_rot[1, :]
#         self.pos_h_zc = v_rot[2, :]
        
#         self.pos_h_xc_preys = v_rot_preys[0, :]
#         self.pos_h_yc_preys = v_rot_preys[1, :]
#         self.pos_h_zc_preys = v_rot_preys[2, :]

#     def plot_swarm(self, pos_xs, pos_ys, pos_zs, pos_hxs, pos_hys, pos_hzs, pos_preys_xs, pos_preys_ys, pos_preys_zs, pos_preys_hxs, pos_preys_hys, pos_preys_hzs):
#         """Update the swarm plot with current positions."""
#         self.plotter.update_plot(pos_xs, pos_ys, pos_zs, pos_hxs, pos_hys, pos_hzs, pos_preys_xs, pos_preys_ys, pos_preys_zs, pos_preys_hxs, pos_preys_hys, pos_preys_hzs)
import numpy as np
from swarmPlot import SwarmPlotter

class FlockingUtils:
    def __init__(self, 
                 n_predators, 
                 n_prey, 
                 center_x_predators, 
                 center_y_predators, 
                 center_z_predators, 
                 spacing, 
                 center_x_prey, 
                 center_y_prey, 
                 center_z_prey, 
                 drones_ids, 
                 boundless,
                 _3D: bool = False,
                 d_des: float = None,
                 sensing_range_own: float = None,
                 sensing_range_other: float = None,
                 repulsion_strength: float = None,
                 alpha_strength: float = None,
                 epsilon_strength: float = None,
                 adaptive_sigma_strength: float = None):
        """
        Initialize flocking utilities with unified symmetric parameters.
        
        Parameters:
        -----------
        d_des : float
            Desired inter-agent distance (used to compute sigma = d_des/√2)
        sensing_range_own : float
            Sensing range for agents within own swarm
        sensing_range_other : float
            Sensing range for detecting agents from other swarm
        repulsion_strength : float
            Strength of repulsion from other swarm (kappa)
        alpha_strength : float
            Attraction/repulsion force multiplier
        epsilon_strength : float
            Potential field depth parameter
        """
        self.boundless = boundless
        self._3D = _3D
        self.n_predators = n_predators
        self.n_prey = n_prey
        
        # Predator initialization
        self.center_x_predators = center_x_predators
        self.center_y_predators = center_y_predators
        self.center_z_predators = center_z_predators
        
        # Prey initialization
        self.center_x_prey = center_x_prey
        self.center_y_prey = center_y_prey
        self.center_z_prey = center_z_prey
        
        self.spacing = spacing
        self.drones_ids = drones_ids
        
        # ============================================================
        # UNIFIED PARAMETERS - SAME FOR BOTH PREDATORS AND PREY
        # ============================================================
        
        # Default parameters based on mode
        if not _3D:
            if self.boundless:
                # Boundless 2D defaults
                default_d_des = 1.2
                default_sensing_range_own = 3.0
                default_sensing_range_other = 3.0
                default_repulsion_strength = 1.5
                default_alpha = 2.0
                default_epsilon = 12.0
            else:
                # Bounded 2D defaults
                default_d_des = 0.707  # sqrt(0.5)
                default_sensing_range_own = 2.0
                default_sensing_range_other = 2.0
                default_repulsion_strength = 1.5
                default_alpha = 2.0
                default_epsilon = 12.0
        else:
            # 3D defaults
            default_d_des = 1.4 * np.sqrt(2)
            default_sensing_range_own = 4.0
            default_sensing_range_other = 3.0
            default_repulsion_strength = 1.5
            default_alpha = 3.0
            default_epsilon = 12.0
        
        # Core flocking parameters (unified for both swarms)
        self.d_des = d_des if d_des is not None else default_d_des
        self.sigma = self.d_des / np.sqrt(2)  # Derived from desired distance
        
        self.sensing_range_own = sensing_range_own if sensing_range_own is not None else default_sensing_range_own
        self.sensing_range_other = sensing_range_other if sensing_range_other is not None else default_sensing_range_other
        self.repulsion_strength = repulsion_strength if repulsion_strength is not None else default_repulsion_strength
        
        # Adaptive sigma strength for predators (tightens formation when prey nearby)
        # Higher values = more aggressive pursuit behavior
        if _3D:
            self.adaptive_sigma_strength = 4.5
        else:
            if self.boundless:
                self.adaptive_sigma_strength = 1.5
            else:
                self.adaptive_sigma_strength = 0.3
        
        # Force parameters
        self.alpha = alpha_strength if alpha_strength is not None else default_alpha
        self.epsilon = epsilon_strength if epsilon_strength is not None else default_epsilon
        self.beta = 2.0  # Alignment force strength (currently unused)
        
        # Motion parameters
        self.k1 = 0.5  # Linear velocity gain
        self.k2 = 0.1  # Angular velocity gain
        self.max_speed = 0.20  # Maximum linear velocity
        self.max_angular_speed = 1.5708  # Maximum angular velocity (π/2)
        
        # Noise and dynamics
        self.dt = 0.05
        self.noise_pos = 0.03
        self.noise_heading = np.pi / 90
        self.rng = np.random.default_rng(1234)
        self.mean_noise = 0.3
        
        # Boundary parameters
        self.boundary_threshold = 0.5
        self.boundary_repulsion_decay = 1.0
        self.boundary_repulsion_strength = 2.0
        self.boundary_min_distance = 0.5
        self.boundary_x = 7.0
        self.boundary_y = 5.0
        self.boundary_z = 5.0
        
        # Smoothing parameters for stable motion
        self.heading_smoothing = 0.8  # Higher = smoother heading changes
        self.velocity_smoothing = 0.7  # Higher = smoother velocity changes
        
        # Alignment control
        self.use_heading_alignment = False
        
        # Initialize sigma arrays (will be updated adaptively)
        self.sigma_predators = np.full(self.n_predators, self.sigma)
        self.sigma_prey = self.sigma  # Prey use constant sigma
        
        # Initialize arrays for predators
        self.d_ij_predators = np.zeros([self.n_predators, self.n_predators])
        self.heading_x_predators = np.zeros(self.n_predators)
        self.heading_y_predators = np.zeros(self.n_predators)
        self.heading_z_predators = np.zeros(self.n_predators)
        self.angle_ij_x_predators = np.zeros([self.n_predators, self.n_predators])
        self.angle_ij_y_predators = np.zeros([self.n_predators, self.n_predators])
        self.angle_ij_z_predators = np.zeros([self.n_predators, self.n_predators])
        self.force_x_predators = np.zeros(self.n_predators)
        self.force_y_predators = np.zeros(self.n_predators)
        self.force_z_predators = np.zeros(self.n_predators)
        self.velocity_predators = np.zeros(self.n_predators)
        self.angular_velocity_predators = np.zeros(self.n_predators)
        
        # Previous velocities for smoothing
        self.prev_velocity_predators = np.zeros(self.n_predators)
        self.prev_angular_velocity_predators = np.zeros(self.n_predators)
        
        # Initialize arrays for prey
        self.d_ij_prey = np.zeros([self.n_prey, self.n_prey])
        self.heading_x_prey = np.zeros(self.n_prey)
        self.heading_y_prey = np.zeros(self.n_prey)
        self.heading_z_prey = np.zeros(self.n_prey)
        self.angle_ij_x_prey = np.zeros([self.n_prey, self.n_prey])
        self.angle_ij_y_prey = np.zeros([self.n_prey, self.n_prey])
        self.angle_ij_z_prey = np.zeros([self.n_prey, self.n_prey])
        self.force_x_prey = np.zeros(self.n_prey)
        self.force_y_prey = np.zeros(self.n_prey)
        self.force_z_prey = np.zeros(self.n_prey)
        self.velocity_prey = np.zeros(self.n_prey)
        self.angular_velocity_prey = np.zeros(self.n_prey)
        
        # Previous velocities for smoothing
        self.prev_velocity_prey = np.zeros(self.n_prey)
        self.prev_angular_velocity_prey = np.zeros(self.n_prey)
        
        # Cross-swarm distance matrices
        self.d_predators_to_prey = np.zeros([self.n_predators, self.n_prey])
        self.d_prey_to_predators = np.zeros([self.n_prey, self.n_predators])
        
        # Plotter
        self.plotter = SwarmPlotter(self.n_predators, 
                                    self.n_prey, 
                                    self.boundary_x, 
                                    self.boundary_y, 
                                    self.boundary_z, 
                                    no_sensor_percentage=0.0,  # All agents have sensors
                                    boundless=self.boundless)
    
    def _calculate_grid_dimensions(self, num_agents):
        """Calculate optimal grid dimensions for initialization."""
        # Try to find perfect rectangular fit
        for x in range(int(np.ceil(np.sqrt(num_agents))), 0, -1):
            if num_agents % x == 0:
                return (x, num_agents // x)
        
        # Fall back to square-like configuration
        square_root = round(num_agents ** 0.5)
        best_diff = float('inf')
        dimensions = (1, num_agents)
        for x in range(square_root, 0, -1):
            y = int(np.ceil(num_agents / x))
            diff = abs(x - y)
            if diff < best_diff and x * y >= num_agents:
                best_diff = diff
                dimensions = (x, y)
        return dimensions
    
    def initialize_positions(self, is_predator: bool = True):
        """
        Initialize agent positions in a grid formation.
        
        Parameters:
        -----------
        is_predator : bool
            True for predators, False for prey
        """
        if is_predator:
            num_agents = self.n_predators
            center = (self.center_x_predators, self.center_y_predators, self.center_z_predators)
        else:
            num_agents = self.n_prey
            center = (self.center_x_prey, self.center_y_prey, self.center_z_prey)
        
        # Calculate grid dimensions based on larger swarm
        max_agents = max(self.n_predators, self.n_prey)
        dimensions = self._calculate_grid_dimensions(max_agents)
        
        # Generate grid positions
        grid_positions = np.mgrid[0:dimensions[0], 0:dimensions[1]].reshape(2, -1).T
        grid_positions = grid_positions * self.spacing
        
        # Center the grid
        offset = np.array(center[:2]) - (np.array(dimensions) * self.spacing / 2)
        grid_positions += offset
        
        # Add noise for natural variation
        noise = np.random.normal(loc=self.mean_noise, scale=self.mean_noise / 3, 
                                size=grid_positions.shape)
        grid_positions += noise
        
        # Set z positions
        z_positions = np.full(num_agents, center[2])
        
        # Initialize random heading directions
        theta = np.random.uniform(0, 2 * np.pi, num_agents)
        phi = np.random.uniform(0, np.pi, num_agents) if self._3D else np.full(num_agents, np.pi/2)
        
        heading_x = np.sin(phi) * np.cos(theta)
        heading_y = np.sin(phi) * np.sin(theta)
        heading_z = np.cos(phi)
        
        if is_predator:
            self.heading_x_predators = heading_x
            self.heading_y_predators = heading_y
            self.heading_z_predators = heading_z
        else:
            self.heading_x_prey = heading_x
            self.heading_y_prey = heading_y
            self.heading_z_prey = heading_z
        
        return (grid_positions[:num_agents, 0], 
                grid_positions[:num_agents, 1], 
                z_positions,
                heading_x, heading_y, heading_z)
    
    def initialize_positions3D(self, preds: bool = True):
        """Wrapper for 3D initialization compatibility."""
        return self.initialize_positions(is_predator=preds)
    
    def calc_dij(self, pos_x_predators, pos_y_predators, pos_z_predators, 
                 pos_x_prey, pos_y_prey, pos_z_prey):
        """Calculate all pairwise distances."""
        # Within-swarm distances for predators
        self.d_ij_predators = np.sqrt(
            (pos_x_predators[:, None] - pos_x_predators)**2 + 
            (pos_y_predators[:, None] - pos_y_predators)**2 + 
            (pos_z_predators[:, None] - pos_z_predators)**2
        )
        self.d_ij_predators[(self.d_ij_predators > self.sensing_range_own) | 
                            (self.d_ij_predators == 0)] = np.inf
        
        # Within-swarm distances for prey
        self.d_ij_prey = np.sqrt(
            (pos_x_prey[:, None] - pos_x_prey)**2 + 
            (pos_y_prey[:, None] - pos_y_prey)**2 + 
            (pos_z_prey[:, None] - pos_z_prey)**2
        )
        self.d_ij_prey[(self.d_ij_prey > self.sensing_range_own) | 
                       (self.d_ij_prey == 0)] = np.inf
        
        # Cross-swarm distances
        self.d_predators_to_prey = np.sqrt(
            (pos_x_predators[:, None] - pos_x_prey)**2 + 
            (pos_y_predators[:, None] - pos_y_prey)**2 + 
            (pos_z_predators[:, None] - pos_z_prey)**2
        )
        self.d_prey_to_predators = self.d_predators_to_prey.T
    
    def calc_ang_ij(self, pos_x_predators, pos_y_predators, pos_z_predators,
                    pos_x_prey, pos_y_prey, pos_z_prey):
        """Calculate angular relationships between agents."""
        # Predators
        with np.errstate(invalid='ignore'):
            self.angle_ij_x_predators = np.arccos(
                np.clip((pos_x_predators - pos_x_predators[:, None]) / self.d_ij_predators, -1, 1))
            self.angle_ij_y_predators = np.arccos(
                np.clip((pos_y_predators - pos_y_predators[:, None]) / self.d_ij_predators, -1, 1))
            self.angle_ij_z_predators = np.arccos(
                np.clip((pos_z_predators - pos_z_predators[:, None]) / self.d_ij_predators, -1, 1))
        
        # Prey
        with np.errstate(invalid='ignore'):
            self.angle_ij_x_prey = np.arccos(
                np.clip((pos_x_prey - pos_x_prey[:, None]) / self.d_ij_prey, -1, 1))
            self.angle_ij_y_prey = np.arccos(
                np.clip((pos_y_prey - pos_y_prey[:, None]) / self.d_ij_prey, -1, 1))
            self.angle_ij_z_prey = np.arccos(
                np.clip((pos_z_prey - pos_z_prey[:, None]) / self.d_ij_prey, -1, 1))
    
    def calc_grad_vals(self, pos_x_predators, pos_y_predators, pos_z_predators,
                       pos_x_prey, pos_y_prey, pos_z_prey, _3D):
        """
        Calculate adaptive sigma values for predators based on proximity to prey.
        This makes predators more aggressive (tighter formation) when prey are nearby.
        """
        # Calculate cross-swarm distance matrix
        self.d_predators_to_prey = np.sqrt(
            (pos_x_predators[:, None] - pos_x_prey)**2 + 
            (pos_y_predators[:, None] - pos_y_prey)**2 + 
            (pos_z_predators[:, None] - pos_z_prey)**2
        )
        self.d_prey_to_predators = self.d_predators_to_prey.T
        
        # Mark distances outside sensing range as infinite
        distance_matrix = self.d_predators_to_prey.copy()
        distance_matrix[distance_matrix > self.sensing_range_other] = np.inf
        
        # Calculate average distance to nearby prey for each predator
        avg_distances_to_prey = np.zeros(self.n_predators)
        for i in range(self.n_predators):
            within_range = distance_matrix[i][distance_matrix[i] != np.inf]
            if within_range.size > 0:
                avg_distances_to_prey[i] = np.mean(within_range)
            else:
                avg_distances_to_prey[i] = np.inf
        
        # Adaptive sigma for predators: tighter formation when prey nearby
        # Formula: σ_adaptive = σ_base + adaptive_strength / distance_to_prey
        
        # Update predator sigma based on proximity to prey
        self.sigma_predators = np.where(
            avg_distances_to_prey == np.inf,
            self.sigma,  # No prey nearby: use base sigma
            self.sigma + self.adaptive_sigma_strength / avg_distances_to_prey  # Prey nearby: tighten formation
        )
    
    def calc_p_forcesADM(self):
        """Calculate ADM (Attractive-Dissipative-Milling) forces for predators with adaptive sigma."""
        # Reset forces
        self.force_x_predators = np.zeros(self.n_predators)
        self.force_y_predators = np.zeros(self.n_predators)
        self.force_z_predators = np.zeros(self.n_predators)
        
        # Calculate potential-based forces using adaptive sigma for each predator
        with np.errstate(invalid='ignore', divide='ignore'):
            # Use adaptive sigma (different for each predator based on prey proximity)
            forces = -self.epsilon * (
                self.sigma_predators[:, np.newaxis] / self.d_ij_predators - 
                np.sqrt(self.sigma_predators[:, np.newaxis] / self.d_ij_predators)
            )
            forces = np.nan_to_num(forces, nan=0.0, posinf=0.0, neginf=0.0)
        
        # Sum forces in each direction
        self.force_x_predators = self.alpha * np.nansum(
            forces * np.cos(self.angle_ij_x_predators), axis=1)
        self.force_y_predators = self.alpha * np.nansum(
            forces * np.cos(self.angle_ij_y_predators), axis=1)
        self.force_z_predators = self.alpha * np.nansum(
            forces * np.cos(self.angle_ij_z_predators), axis=1)
        
        # Prevent zero forces
        self.force_x_predators = np.where(self.force_x_predators == 0, 0.00001, self.force_x_predators)
        self.force_y_predators = np.where(self.force_y_predators == 0, 0.00001, self.force_y_predators)
        self.force_z_predators = np.where(self.force_z_predators == 0, 0.00001, self.force_z_predators)
    
    def calc_p_forces(self):
        """Calculate ADM forces for prey (uses constant sigma - no adaptation)."""
        # Reset forces
        self.force_x_prey = np.zeros(self.n_prey)
        self.force_y_prey = np.zeros(self.n_prey)
        self.force_z_prey = np.zeros(self.n_prey)
        
        # Calculate potential-based forces (prey use constant sigma)
        with np.errstate(invalid='ignore', divide='ignore'):
            forces = -self.epsilon * (
                self.sigma_prey / self.d_ij_prey - 
                np.sqrt(self.sigma_prey / self.d_ij_prey)
            )
            forces = np.nan_to_num(forces, nan=0.0, posinf=0.0, neginf=0.0)
        
        # Sum forces in each direction
        self.force_x_prey = self.alpha * np.nansum(
            forces * np.cos(self.angle_ij_x_prey), axis=1)
        self.force_y_prey = self.alpha * np.nansum(
            forces * np.cos(self.angle_ij_y_prey), axis=1)
        self.force_z_prey = self.alpha * np.nansum(
            forces * np.cos(self.angle_ij_z_prey), axis=1)
        
        # Prevent zero forces
        self.force_x_prey = np.where(self.force_x_prey == 0, 0.00001, self.force_x_prey)
        self.force_y_prey = np.where(self.force_y_prey == 0, 0.00001, self.force_y_prey)
        self.force_z_prey = np.where(self.force_z_prey == 0, 0.00001, self.force_z_prey)
    
    def calc_repulsion_predator_forces(self, pos_x_predators, pos_y_predators, pos_z_predators,
                                       pos_x_prey, pos_y_prey, pos_z_prey):
        """Calculate repulsion forces on prey from predators."""
        positions_prey = np.column_stack((pos_x_prey, pos_y_prey, pos_z_prey))
        positions_predators = np.column_stack((pos_x_predators, pos_y_predators, pos_z_predators))
        
        # Distance matrix (already calculated in calc_dij)
        distance_matrix = self.d_prey_to_predators
        within_range = distance_matrix <= self.sensing_range_other
        
        # Calculate repulsion for each prey
        for i in range(self.n_prey):
            nearby_indices = np.where(within_range[i])[0]
            
            if nearby_indices.size > 0:
                # Average position of nearby predators
                avg_predator_pos = np.mean(positions_predators[nearby_indices], axis=0)
                
                # Direction away from predators
                direction = positions_prey[i] - avg_predator_pos
                distance = np.linalg.norm(direction)
                
                if distance > 0:
                    # Adaptive repulsion strength (stronger when closer)
                    kappa = self.repulsion_strength / (distance + 0.1)
                    
                    # Add repulsion force
                    self.force_x_prey[i] += kappa * direction[0]
                    self.force_y_prey[i] += kappa * direction[1]
                    self.force_z_prey[i] += kappa * direction[2]
    
    def calc_boun_rep(self, pos_x, pos_y, pos_z, heading_x, heading_y, heading_z, 
                      preds: bool):
        """Calculate boundary repulsion forces."""
        if self.boundless:
            return  # No boundary forces in boundless mode
        
        position_x = pos_x
        position_y = pos_y
        
        # Calculate distances to boundaries
        dist_to_left = position_x
        dist_to_right = self.boundary_x - position_x
        dist_to_bottom = position_y
        dist_to_top = self.boundary_y - position_y
        
        # Initialize repulsion vectors
        repulsion_x = np.zeros_like(position_x)
        repulsion_y = np.zeros_like(position_y)
        
        # Apply repulsion for each boundary
        for i in range(len(position_x)):
            # Left boundary
            if dist_to_left[i] < self.boundary_repulsion_decay:
                strength = self.boundary_repulsion_strength * (
                    1.0 / max(dist_to_left[i], self.boundary_min_distance) - 
                    1.0 / self.boundary_repulsion_decay
                )
                repulsion_x[i] += strength
            
            # Right boundary
            if dist_to_right[i] < self.boundary_repulsion_decay:
                strength = self.boundary_repulsion_strength * (
                    1.0 / max(dist_to_right[i], self.boundary_min_distance) - 
                    1.0 / self.boundary_repulsion_decay
                )
                repulsion_x[i] -= strength
            
            # Bottom boundary
            if dist_to_bottom[i] < self.boundary_repulsion_decay:
                strength = self.boundary_repulsion_strength * (
                    1.0 / max(dist_to_bottom[i], self.boundary_min_distance) - 
                    1.0 / self.boundary_repulsion_decay
                )
                repulsion_y[i] += strength
            
            # Top boundary
            if dist_to_top[i] < self.boundary_repulsion_decay:
                strength = self.boundary_repulsion_strength * (
                    1.0 / max(dist_to_top[i], self.boundary_min_distance) - 
                    1.0 / self.boundary_repulsion_decay
                )
                repulsion_y[i] -= strength
        
        # Add boundary forces to total forces
        if preds:
            self.force_x_predators += repulsion_x
            self.force_y_predators += repulsion_y
        else:
            self.force_x_prey += repulsion_x
            self.force_y_prey += repulsion_y
    
    def calc_u_w(self):
        """Calculate linear and angular velocities with smoothing."""
        # Predators
        force_mag_pred = np.sqrt(
            self.force_x_predators**2 + 
            self.force_y_predators**2 + 
            self.force_z_predators**2
        )
        force_mag_pred = np.where(force_mag_pred == 0, 0.00001, force_mag_pred)
        
        dot_product_pred = (
            self.force_x_predators * self.heading_x_predators + 
            self.force_y_predators * self.heading_y_predators + 
            self.force_z_predators * self.heading_z_predators
        )
        cos_angle_pred = np.clip(dot_product_pred / force_mag_pred, -1.0, 1.0)
        angle_pred = np.arccos(cos_angle_pred)
        
        # Calculate raw velocities
        raw_velocity_pred = self.k1 * force_mag_pred * np.cos(angle_pred) + 0.05
        raw_angular_pred = self.k2 * force_mag_pred * np.sin(angle_pred)
        
        # Apply smoothing
        self.velocity_predators = (
            self.velocity_smoothing * self.prev_velocity_predators + 
            (1 - self.velocity_smoothing) * raw_velocity_pred
        )
        self.angular_velocity_predators = (
            self.heading_smoothing * self.prev_angular_velocity_predators + 
            (1 - self.heading_smoothing) * raw_angular_pred
        )
        
        # Clip to limits
        self.velocity_predators = np.clip(self.velocity_predators, 0, self.max_speed)
        self.angular_velocity_predators = np.clip(
            self.angular_velocity_predators, -self.max_angular_speed, self.max_angular_speed
        )
        
        # Update previous values
        self.prev_velocity_predators = self.velocity_predators.copy()
        self.prev_angular_velocity_predators = self.angular_velocity_predators.copy()
        
        # Prey (identical calculation)
        force_mag_prey = np.sqrt(
            self.force_x_prey**2 + 
            self.force_y_prey**2 + 
            self.force_z_prey**2
        )
        force_mag_prey = np.where(force_mag_prey == 0, 0.00001, force_mag_prey)
        
        dot_product_prey = (
            self.force_x_prey * self.heading_x_prey + 
            self.force_y_prey * self.heading_y_prey + 
            self.force_z_prey * self.heading_z_prey
        )
        cos_angle_prey = np.clip(dot_product_prey / force_mag_prey, -1.0, 1.0)
        angle_prey = np.arccos(cos_angle_prey)
        
        # Calculate raw velocities
        raw_velocity_prey = self.k1 * force_mag_prey * np.cos(angle_prey) + 0.05
        raw_angular_prey = self.k2 * force_mag_prey * np.sin(angle_prey)
        
        # Apply smoothing
        self.velocity_prey = (
            self.velocity_smoothing * self.prev_velocity_prey + 
            (1 - self.velocity_smoothing) * raw_velocity_prey
        )
        self.angular_velocity_prey = (
            self.heading_smoothing * self.prev_angular_velocity_prey + 
            (1 - self.heading_smoothing) * raw_angular_prey
        )
        
        # Clip to limits
        self.velocity_prey = np.clip(self.velocity_prey, 0, self.max_speed)
        self.angular_velocity_prey = np.clip(
            self.angular_velocity_prey, -self.max_angular_speed, self.max_angular_speed
        )
        
        # Update previous values
        self.prev_velocity_prey = self.velocity_prey.copy()
        self.prev_angular_velocity_prey = self.angular_velocity_prey.copy()
        
        return self.velocity_predators, self.velocity_prey
    
    def get_heading(self):
        """Get heading angles from heading vectors."""
        # Predators
        heading_mag_pred = np.sqrt(
            self.heading_x_predators**2 + 
            self.heading_y_predators**2 + 
            self.heading_z_predators**2
        )
        heading_mag_pred = np.where(heading_mag_pred == 0, 1.0, heading_mag_pred)
        
        heading_angles_pred_x = np.arccos(np.clip(self.heading_x_predators / heading_mag_pred, -1, 1))
        heading_angles_pred_y = np.arccos(np.clip(self.heading_y_predators / heading_mag_pred, -1, 1))
        heading_angles_pred_z = np.arccos(np.clip(self.heading_z_predators / heading_mag_pred, -1, 1))
        
        # Prey
        heading_mag_prey = np.sqrt(
            self.heading_x_prey**2 + 
            self.heading_y_prey**2 + 
            self.heading_z_prey**2
        )
        heading_mag_prey = np.where(heading_mag_prey == 0, 1.0, heading_mag_prey)
        
        heading_angles_prey_x = np.arccos(np.clip(self.heading_x_prey / heading_mag_prey, -1, 1))
        heading_angles_prey_y = np.arccos(np.clip(self.heading_y_prey / heading_mag_prey, -1, 1))
        heading_angles_prey_z = np.arccos(np.clip(self.heading_z_prey / heading_mag_prey, -1, 1))
        
        return (heading_angles_pred_x, heading_angles_pred_y, heading_angles_pred_z,
                heading_angles_prey_x, heading_angles_prey_y, heading_angles_prey_z)
    
    def calculate_rotated_vector_batch(self, X1, Y1, Z1, X2, Y2, Z2, angular_velocity_dt):
        """
        Rotate heading vectors using Rodrigues' rotation formula.
        Rotates vector1 towards vector2 by angular_velocity * dt.
        """
        vector1 = np.stack([X1, Y1, Z1], axis=-1)
        vector2 = np.stack([X2, Y2, Z2], axis=-1)
        
        # Normalize vectors
        vector1_mag = np.linalg.norm(vector1, axis=1, keepdims=True)
        vector1_mag = np.where(vector1_mag == 0, 1.0, vector1_mag)
        vector1_normalized = vector1 / vector1_mag
        
        vector2_mag = np.linalg.norm(vector2, axis=1, keepdims=True)
        vector2_mag = np.where(vector2_mag == 0, 1.0, vector2_mag)
        vector2_normalized = vector2 / vector2_mag
        
        # Rotation axis (perpendicular to both vectors)
        rotation_axis = np.cross(vector1_normalized, vector2_normalized)
        axis_mag = np.linalg.norm(rotation_axis, axis=1, keepdims=True)
        
        # Handle parallel vectors
        rotation_axis = np.where(axis_mag > 1e-6, rotation_axis / axis_mag, 
                                np.array([[0, 0, 1]]))
        
        # Rodrigues' rotation formula
        cos_theta = np.cos(angular_velocity_dt)[:, np.newaxis]
        sin_theta = np.sin(angular_velocity_dt)[:, np.newaxis]
        
        k_cross_v = np.cross(rotation_axis, vector1_normalized)
        dot_kv = np.sum(rotation_axis * vector1_normalized, axis=1, keepdims=True)
        
        rotated = (vector1_normalized * cos_theta + 
                  k_cross_v * sin_theta + 
                  rotation_axis * dot_kv * (1 - cos_theta))
        
        # Restore original magnitude
        rotated = rotated * vector1_mag
        
        return rotated.T
    
    def update_heading(self):
        """Update heading vectors based on forces and angular velocities."""
        # Predators
        rotated_pred = self.calculate_rotated_vector_batch(
            self.heading_x_predators, 
            self.heading_y_predators, 
            self.heading_z_predators,
            self.force_x_predators, 
            self.force_y_predators, 
            self.force_z_predators,
            self.angular_velocity_predators * self.dt
        )
        
        self.heading_x_predators = rotated_pred[0, :]
        self.heading_y_predators = rotated_pred[1, :]
        self.heading_z_predators = rotated_pred[2, :]
        
        # Normalize to maintain unit vectors
        mag_pred = np.sqrt(
            self.heading_x_predators**2 + 
            self.heading_y_predators**2 + 
            self.heading_z_predators**2
        )
        mag_pred = np.where(mag_pred == 0, 1.0, mag_pred)
        self.heading_x_predators /= mag_pred
        self.heading_y_predators /= mag_pred
        self.heading_z_predators /= mag_pred
        
        # Prey
        rotated_prey = self.calculate_rotated_vector_batch(
            self.heading_x_prey, 
            self.heading_y_prey, 
            self.heading_z_prey,
            self.force_x_prey, 
            self.force_y_prey, 
            self.force_z_prey,
            self.angular_velocity_prey * self.dt
        )
        
        self.heading_x_prey = rotated_prey[0, :]
        self.heading_y_prey = rotated_prey[1, :]
        self.heading_z_prey = rotated_prey[2, :]
        
        # Normalize to maintain unit vectors
        mag_prey = np.sqrt(
            self.heading_x_prey**2 + 
            self.heading_y_prey**2 + 
            self.heading_z_prey**2
        )
        mag_prey = np.where(mag_prey == 0, 1.0, mag_prey)
        self.heading_x_prey /= mag_prey
        self.heading_y_prey /= mag_prey
        self.heading_z_prey /= mag_prey
    
    def plot_swarm(self, pos_x_pred, pos_y_pred, pos_z_pred, 
                   heading_x_pred, heading_y_pred, heading_z_pred,
                   pos_x_prey, pos_y_prey, pos_z_prey, 
                   heading_x_prey, heading_y_prey, heading_z_prey):
        """Update the swarm visualization."""
        self.plotter.update_plot(
            pos_x_pred, pos_y_pred, pos_z_pred, 
            heading_x_pred, heading_y_pred, heading_z_pred,
            pos_x_prey, pos_y_prey, pos_z_prey, 
            heading_x_prey, heading_y_prey, heading_z_prey
        )