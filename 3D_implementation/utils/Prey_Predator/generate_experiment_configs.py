"""
Generate experiment configurations for systematic parameter study.

This script creates 9 configurations testing the combinations of:
- Desired Distance (d_des): 0.7, 1.2, 2.0
- Sensing Range: 0.8*1.8*d_des, 1.8*d_des, 1.2*1.8*d_des
- Swarm Size (n_s): 5, 10, 20

Usage:
    python generate_experiment_configs.py
    python run2.py --config experiments_config.json --run_all_sets
"""

import json
import numpy as np

# Experimental parameters
D_DES_VALUES = [0.7, 1.2, 2.0]
SWARM_SIZES = [5, 10, 20]

# Sensing range multipliers (relative to 1.8*d_des)
# less than, equal to, greater than
SENSING_MULTIPLIERS = {
    'less': 0.8,      # 80% of 1.8*d_des
    'equal': 1.0,     # exactly 1.8*d_des
    'greater': 1.2    # 120% of 1.8*d_des
}

def generate_experiment_name(d_des, sensing_type, n_s):
    """Generate a descriptive experiment name."""
    return f"d{d_des:.1f}_s{sensing_type}_n{n_s}"

def generate_experiment_description(d_des, sensing_range, sensing_type, n_s):
    """Generate a detailed description."""
    sigma = d_des / np.sqrt(2)
    return (f"d_des={d_des}, σ={sigma:.3f}, sensing={sensing_type} "
            f"({sensing_range:.2f}m = {sensing_type} 1.8×d_des), n={n_s}")

def create_base_config():
    """Create the base configuration common to all experiments."""
    return {
        "n_predators": 10,  # Will be overridden
        "n_prey": 10,       # Will be overridden
        "3d": False,
        "boundless": True,
        "num_trials": 3,
        "duration_sec": 200,
        "analysis_window_sec": 150,
        "gui": False,
        
        "simulation": {
            "default_simulation_freq_hz": 240,
            "default_control_freq_hz": 30
        },
        
        "flocking_parameters": {
            "desired_distance": 1.2,           # Will be overridden
            "sensing_range_own_swarm": 3.0,    # Will be overridden
            "sensing_range_other_swarm": 3.0,  # Will be overridden
            "repulsion_strength": 1.5,
            "alpha": 2.0,
            "epsilon": 12.0,
            "adaptive_sigma_strength": 1.5
        },
        
        "behavior_analysis": {
            "follow_distance_threshold": 4.0,
            "engulf_distance_threshold": 2.0,
            "separation_distance_threshold": 8.0,
            "min_behavior_duration": 10
        },
        
        "initialization": {
            "min_distance_3d": 4.0,
            "min_distance_formula_multiplier": 0.15,
            "min_distance_formula_base": 1.2,
            "min_distance_formula_offset": 1.5,
            
            "boundless_settings": {
                "init_center_offset_range": [-1.0, 1.0],
                "spacing": 1.2  # Will be adjusted based on d_des
            },
            
            "bounded_settings": {
                "init_center_x": 3.0,
                "init_center_y": 2.5,
                "init_center_z": 1.0,
                "spacing": 0.8
            }
        }
    }

def generate_all_experiments():
    """Generate all 27 experiment configurations (3×3×3)."""
    
    base_config = create_base_config()
    parameter_sets = []
    
    experiment_count = 0
    
    for d_des in D_DES_VALUES:
        for sensing_name, sensing_mult in SENSING_MULTIPLIERS.items():
            for n_s in SWARM_SIZES:
                experiment_count += 1
                
                # Calculate sensing range
                sensing_range = 1.8 * d_des * sensing_mult
                
                # Calculate sigma
                sigma = d_des / np.sqrt(2)
                
                # Generate experiment name
                exp_name = generate_experiment_name(d_des, sensing_name, n_s)
                
                # Generate description
                exp_desc = generate_experiment_description(
                    d_des, sensing_range, sensing_name, n_s
                )
                
                # Create parameter overrides
                overrides = {
                    "n_predators": n_s,
                    "n_prey": n_s,
                    "flocking_parameters": {
                        "desired_distance": d_des,
                        "sensing_range_own_swarm": sensing_range,
                        "sensing_range_other_swarm": sensing_range,
                    },
                    "initialization": {
                        "boundless_settings": {
                            "spacing": d_des  # Use d_des as spacing
                        }
                    }
                }
                
                # Create parameter set entry
                parameter_set = {
                    "name": exp_name,
                    "description": exp_desc,
                    "overrides": overrides,
                    "_metadata": {
                        "d_des": d_des,
                        "sigma": round(sigma, 4),
                        "sensing_range": round(sensing_range, 4),
                        "sensing_type": sensing_name,
                        "sensing_multiplier": sensing_mult,
                        "target_ratio": 1.8,
                        "actual_ratio": round(sensing_range / d_des, 4),
                        "swarm_size": n_s,
                        "experiment_number": experiment_count
                    }
                }
                
                parameter_sets.append(parameter_set)
    
    # Create final configuration
    config = {
        "base_config": base_config,
        "parameter_sets": parameter_sets,
        "_experiment_info": {
            "total_experiments": experiment_count,
            "description": "Systematic study of d_des, sensing_range, and swarm_size",
            "d_des_values": D_DES_VALUES,
            "sensing_multipliers": SENSING_MULTIPLIERS,
            "swarm_sizes": SWARM_SIZES,
            "design": "3×3×3 full factorial design"
        }
    }
    
    return config

def save_config(config, filename="experiments_config.json"):
    """Save configuration to JSON file."""
    with open(filename, 'w') as f:
        json.dump(config, f, indent=2)
    print(f"✓ Configuration saved to: {filename}")

def print_experiment_summary(config):
    """Print a summary of all experiments."""
    print("\n" + "="*80)
    print("EXPERIMENT CONFIGURATION SUMMARY")
    print("="*80)
    
    exp_info = config['_experiment_info']
    print(f"\nDesign: {exp_info['design']}")
    print(f"Total Experiments: {exp_info['total_experiments']}")
    print(f"\nFactors:")
    print(f"  1. Desired Distance (d_des): {exp_info['d_des_values']}")
    print(f"  2. Sensing Range (relative to 1.8×d_des):")
    for name, mult in exp_info['sensing_multipliers'].items():
        print(f"     - {name}: {mult}× (= {mult*1.8}×d_des)")
    print(f"  3. Swarm Size (n_s): {exp_info['swarm_sizes']}")
    
    print(f"\n{'#':<4} {'Name':<20} {'d_des':<8} {'Sensing':<10} {'n_s':<6} {'σ':<8}")
    print("-"*80)
    
    for i, pset in enumerate(config['parameter_sets'], 1):
        meta = pset['_metadata']
        print(f"{i:<4} {pset['name']:<20} {meta['d_des']:<8.1f} "
              f"{meta['sensing_range']:<10.3f} {meta['swarm_size']:<6} "
              f"{meta['sigma']:<8.4f}")
    
    print("="*80)
    print(f"\nTo run all experiments:")
    print(f"  python run2.py --config experiments_config.json --run_all_sets")
    print(f"\nTo run a specific experiment (e.g., #5):")
    exp_name = config['parameter_sets'][4]['name']
    print(f"  python run2.py --config experiments_config.json --parameter_set {exp_name}")
    print(f"\nTo list all experiments:")
    print(f"  python run2.py --config experiments_config.json --list_sets")
    print("="*80 + "\n")

def create_experiment_matrix_table():
    """Create a visual matrix showing all combinations."""
    print("\nEXPERIMENT MATRIX:")
    print("="*80)
    
    for n_s in SWARM_SIZES:
        print(f"\nSwarm Size n_s = {n_s}:")
        print(f"{'d_des':<10} {'Less (<1.8×d)':<20} {'Equal (1.8×d)':<20} {'Greater (>1.8×d)':<20}")
        print("-"*70)
        
        for d_des in D_DES_VALUES:
            row = f"{d_des:<10.1f}"
            for sensing_name in ['less', 'equal', 'greater']:
                sensing_mult = SENSING_MULTIPLIERS[sensing_name]
                sensing_range = 1.8 * d_des * sensing_mult
                exp_name = generate_experiment_name(d_des, sensing_name, n_s)
                row += f" {exp_name:<20}"
            print(row)
    
    print("="*80)

def main():
    """Main function to generate and save experiment configurations."""
    print("\n" + "="*80)
    print("GENERATING EXPERIMENT CONFIGURATIONS")
    print("="*80)
    
    # Generate configurations
    config = generate_all_experiments()
    
    # Save to file
    save_config(config)
    
    # Print summary
    print_experiment_summary(config)
    
    # Print matrix
    create_experiment_matrix_table()
    
    # Print parameter details
    print("\nPARAMETER RELATIONSHIPS:")
    print("="*80)
    print(f"{'d_des':<8} {'σ (=d/√2)':<12} {'Sensing (0.8×1.8×d)':<20} "
          f"{'Sensing (1.8×d)':<16} {'Sensing (1.2×1.8×d)':<20}")
    print("-"*80)
    
    for d_des in D_DES_VALUES:
        sigma = d_des / np.sqrt(2)
        sensing_less = 1.8 * d_des * 0.8
        sensing_equal = 1.8 * d_des * 1.0
        sensing_greater = 1.8 * d_des * 1.2
        
        print(f"{d_des:<8.1f} {sigma:<12.4f} {sensing_less:<20.3f} "
              f"{sensing_equal:<16.3f} {sensing_greater:<20.3f}")
    
    print("="*80)
    
    print("\n✓ Ready to run experiments!")
    print("  Next step: python run2.py --config experiments_config.json --run_all_sets\n")

if __name__ == "__main__":
    main()