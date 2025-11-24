"""
Minimal example showing how to use the new unified flocking system.
This demonstrates the key changes and how to initialize a simulation.
"""

from flocking import FlockingUtils
import numpy as np

# ============================================================
# EXAMPLE 1: Basic initialization with default parameters
# ============================================================

# Create flocking utilities with defaults (boundless 2D mode)
f_util_default = FlockingUtils(
    n_predators=10,
    n_prey=10,
    center_x_predators=3.0,
    center_y_predators=3.0,
    center_z_predators=1.0,
    spacing=1.2,
    center_x_prey=5.0,
    center_y_prey=5.0,
    center_z_prey=1.0,
    drones_ids=list(range(10)),
    boundless=True,
    _3D=False
)

print("Example 1: Default parameters")
print(f"  Desired distance (d_des): {f_util_default.d_des}")
print(f"  Sigma (σ = d_des/√2): {f_util_default.sigma:.3f}")
print(f"  Sensing range (own swarm): {f_util_default.sensing_range_own}")
print(f"  Sensing range (other swarm): {f_util_default.sensing_range_other}")
print(f"  Repulsion strength: {f_util_default.repulsion_strength}")
print(f"  Alpha: {f_util_default.alpha}")
print(f"  Epsilon: {f_util_default.epsilon}")
print()

# ============================================================
# EXAMPLE 2: Custom parameters for tight cohesion
# ============================================================

f_util_tight = FlockingUtils(
    n_predators=15,
    n_prey=15,
    center_x_predators=3.0,
    center_y_predators=3.0,
    center_z_predators=1.0,
    spacing=0.8,
    center_x_prey=6.0,
    center_y_prey=3.0,
    center_z_prey=1.0,
    drones_ids=list(range(15)),
    boundless=True,
    _3D=False,
    # Custom parameters for tight, cohesive swarms
    d_des=0.8,                      # Smaller desired distance
    sensing_range_own=2.5,          # Smaller sensing range
    sensing_range_other=3.0,        # Normal cross-swarm sensing
    repulsion_strength=2.0,         # Stronger repulsion
    alpha_strength=2.5,             # Stronger attraction
    epsilon_strength=15.0           # Deeper potential well
)

print("Example 2: Tight cohesion parameters")
print(f"  Desired distance (d_des): {f_util_tight.d_des}")
print(f"  Sigma (σ): {f_util_tight.sigma:.3f}")
print(f"  Sensing range (own): {f_util_tight.sensing_range_own}")
print(f"  Alpha: {f_util_tight.alpha}")
print()

# ============================================================
# EXAMPLE 3: Initialize positions and run one step
# ============================================================

# Initialize predator positions
pos_x_pred, pos_y_pred, pos_z_pred, h_x_pred, h_y_pred, h_z_pred = \
    f_util_default.initialize_positions(is_predator=True)

# Initialize prey positions
pos_x_prey, pos_y_prey, pos_z_prey, h_x_prey, h_y_prey, h_z_prey = \
    f_util_default.initialize_positions(is_predator=False)

print("Example 3: Position initialization")
print(f"  Predator positions shape: {pos_x_pred.shape}")
print(f"  First predator at: ({pos_x_pred[0]:.2f}, {pos_y_pred[0]:.2f}, {pos_z_pred[0]:.2f})")
print(f"  Prey positions shape: {pos_x_prey.shape}")
print(f"  First prey at: ({pos_x_prey[0]:.2f}, {pos_y_prey[0]:.2f}, {pos_z_prey[0]:.2f})")
print()

# ============================================================
# EXAMPLE 4: Single simulation step
# ============================================================

# Calculate distances
f_util_default.calc_dij(pos_x_pred, pos_y_pred, pos_z_pred,
                        pos_x_prey, pos_y_prey, pos_z_prey)

# Calculate angles
f_util_default.calc_ang_ij(pos_x_pred, pos_y_pred, pos_z_pred,
                           pos_x_prey, pos_y_prey, pos_z_prey)

# Update gradient values (for cross-swarm sensing)
f_util_default.calc_grad_vals(pos_x_pred, pos_y_pred, pos_z_pred,
                               pos_x_prey, pos_y_prey, pos_z_prey, _3D=False)

# Calculate flocking forces
f_util_default.calc_p_forces()      # Prey forces (within-swarm)
f_util_default.calc_p_forcesADM()   # Predator forces (within-swarm)

# Calculate prey repulsion from predators
f_util_default.calc_repulsion_predator_forces(
    pos_x_pred, pos_y_pred, pos_z_pred,
    pos_x_prey, pos_y_prey, pos_z_prey
)

# Calculate velocities (with smoothing)
u_pred, u_prey = f_util_default.calc_u_w()

# Get heading angles
heading_angles = f_util_default.get_heading()

# Update heading vectors
f_util_default.update_heading()

print("Example 4: Single simulation step")
print(f"  Predator velocities: min={u_pred.min():.3f}, max={u_pred.max():.3f}, mean={u_pred.mean():.3f}")
print(f"  Prey velocities: min={u_prey.min():.3f}, max={u_prey.max():.3f}, mean={u_prey.mean():.3f}")
print(f"  Predator forces: ({f_util_default.force_x_predators[0]:.3f}, {f_util_default.force_y_predators[0]:.3f})")
print(f"  Prey forces: ({f_util_default.force_x_prey[0]:.3f}, {f_util_default.force_y_prey[0]:.3f})")
print()

# ============================================================
# EXAMPLE 5: Parameter relationships and recommendations
# ============================================================

print("Example 5: Parameter relationships")
print("  Key relationship: σ = d_des / √2")
print(f"    If d_des = 1.2, then σ = {1.2/np.sqrt(2):.3f}")
print(f"    If d_des = 2.0, then σ = {2.0/np.sqrt(2):.3f}")
print()
print("  Recommendation: sensing_range_own ≥ 2 × d_des")
print(f"    If d_des = 1.2, sensing_range_own should be ≥ {2*1.2:.1f}")
print()
print("  Motion smoothing factors:")
print(f"    Velocity smoothing: {f_util_default.velocity_smoothing} (higher = smoother)")
print(f"    Heading smoothing: {f_util_default.heading_smoothing} (higher = smoother)")
print()

# ============================================================
# EXAMPLE 6: Comparing old vs new parameter values
# ============================================================

print("Example 6: Migration from old parameters")
print("  OLD configuration:")
print("    sigma = 1.2")
print("    Dp = 3.0")
print("    sensing_range = 3.0")
print()
print("  NEW configuration:")
old_sigma = 1.2
new_d_des = old_sigma * np.sqrt(2)
print(f"    desired_distance = {new_d_des:.3f}  (= old_sigma × √2)")
print(f"    sensing_range_own_swarm = 3.0  (= old Dp)")
print(f"    sensing_range_other_swarm = 3.0  (= old sensing_range)")
print()

# ============================================================
# EXAMPLE 7: 3D mode
# ============================================================

f_util_3d = FlockingUtils(
    n_predators=12,
    n_prey=12,
    center_x_predators=3.0,
    center_y_predators=3.0,
    center_z_predators=3.0,
    spacing=1.5,
    center_x_prey=6.0,
    center_y_prey=3.0,
    center_z_prey=3.0,
    drones_ids=list(range(12)),
    boundless=True,
    _3D=True,  # Enable 3D mode
    d_des=1.98,                 # Larger for 3D
    sensing_range_own=4.0,      # Larger for 3D
    alpha_strength=3.0          # Higher for 3D
)

print("Example 7: 3D mode")
print(f"  3D mode enabled: {f_util_3d._3D}")
print(f"  Desired distance: {f_util_3d.d_des}")
print(f"  Sensing range: {f_util_3d.sensing_range_own}")
print(f"  Alpha: {f_util_3d.alpha}")
print()

# ============================================================
# KEY TAKEAWAYS
# ============================================================

print("="*60)
print("KEY TAKEAWAYS")
print("="*60)
print("1. ✅ All agents now have FULL sensing (no non-sensing agents)")
print("2. ✅ Predators and prey use SAME parameters (symmetric)")
print("3. ✅ Sigma is AUTOMATICALLY calculated from d_des")
print("4. ✅ Motion is SMOOTHED for stability")
print("5. ✅ Parameters have CLEAR, descriptive names")
print()
print("Main parameters to tune:")
print("  - desired_distance: Target spacing between agents")
print("  - sensing_range_own_swarm: How far agents detect same swarm")
print("  - sensing_range_other_swarm: How far agents detect other swarm")
print("  - repulsion_strength: How strongly prey avoid predators")
print("  - alpha: Force multiplier for attraction/repulsion")
print("  - epsilon: Depth of potential well")
print("="*60)