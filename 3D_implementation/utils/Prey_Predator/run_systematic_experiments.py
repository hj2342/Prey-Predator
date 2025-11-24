"""
Batch runner for systematic experiments with progress tracking and results organization.

This script:
1. Generates experiment configurations
2. Runs all 27 experiments sequentially
3. Organizes results by parameters
4. Creates summary comparisons

Usage:
    python run_systematic_experiments.py
    python run_systematic_experiments.py --trials 5
    python run_systematic_experiments.py --duration 300
"""

import os
import sys
import json
import argparse
import subprocess
from datetime import datetime
import numpy as np

# Import the config generator
import generate_experiment_configs as gen_config

def create_output_structure(base_dir="systematic_experiments"):
    """Create organized directory structure for results."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    experiment_dir = os.path.join(base_dir, f"run_{timestamp}")
    
    # Create subdirectories
    dirs = {
        'root': experiment_dir,
        'configs': os.path.join(experiment_dir, 'configs'),
        'results': os.path.join(experiment_dir, 'results'),
        'plots': os.path.join(experiment_dir, 'plots'),
        'summaries': os.path.join(experiment_dir, 'summaries'),
        'logs': os.path.join(experiment_dir, 'logs')
    }
    
    for dir_path in dirs.values():
        os.makedirs(dir_path, exist_ok=True)
    
    return dirs

def update_config_with_custom_params(config, trials=None, duration=None):
    """Update configuration with custom trial count and duration."""
    if trials is not None:
        config['base_config']['num_trials'] = trials
    
    if duration is not None:
        config['base_config']['duration_sec'] = duration
    
    return config

def save_experiment_log(dirs, experiment_name, status, metadata=None):
    """Log experiment status and metadata."""
    log_file = os.path.join(dirs['logs'], 'experiment_log.txt')
    
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    with open(log_file, 'a') as f:
        f.write(f"\n{'='*80}\n")
        f.write(f"[{timestamp}] {experiment_name}: {status}\n")
        if metadata:
            for key, value in metadata.items():
                f.write(f"  {key}: {value}\n")
        f.write(f"{'='*80}\n")

def run_single_experiment(config_file, parameter_set_name, output_dir, dirs):
    """Run a single experiment and return status."""
    
    print(f"\n{'='*80}")
    print(f"Running: {parameter_set_name}")
    print(f"{'='*80}")
    
    try:
        # Run the experiment
        cmd = [
            sys.executable,
            'run2.py',
            '--config', config_file,
            '--parameter_set', parameter_set_name,
            '--output_dir', output_dir
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=3600)
        
        if result.returncode == 0:
            print(f"✓ {parameter_set_name} completed successfully")
            save_experiment_log(dirs, parameter_set_name, "SUCCESS")
            return True
        else:
            print(f"✗ {parameter_set_name} failed with return code {result.returncode}")
            print(f"Error output: {result.stderr}")
            save_experiment_log(dirs, parameter_set_name, "FAILED", 
                              {"error": result.stderr[:500]})
            return False
            
    except subprocess.TimeoutExpired:
        print(f"✗ {parameter_set_name} timed out after 1 hour")
        save_experiment_log(dirs, parameter_set_name, "TIMEOUT")
        return False
    except Exception as e:
        print(f"✗ {parameter_set_name} failed with exception: {str(e)}")
        save_experiment_log(dirs, parameter_set_name, "ERROR", {"exception": str(e)})
        return False

def create_comparison_summary(dirs, config):
    """Create a summary comparing all experiments."""
    
    summary_file = os.path.join(dirs['summaries'], 'experiment_comparison.txt')
    
    with open(summary_file, 'w') as f:
        f.write("="*100 + "\n")
        f.write("SYSTEMATIC EXPERIMENT SUMMARY\n")
        f.write("="*100 + "\n\n")
        
        exp_info = config['_experiment_info']
        f.write(f"Design: {exp_info['design']}\n")
        f.write(f"Total Experiments: {exp_info['total_experiments']}\n\n")
        
        f.write("PARAMETER COMBINATIONS:\n")
        f.write("-"*100 + "\n")
        f.write(f"{'#':<5} {'Name':<25} {'d_des':<10} {'σ':<10} {'Sensing':<12} "
                f"{'n_s':<6} {'Sensing/d_des':<15}\n")
        f.write("-"*100 + "\n")
        
        for i, pset in enumerate(config['parameter_sets'], 1):
            meta = pset['_metadata']
            f.write(f"{i:<5} {pset['name']:<25} {meta['d_des']:<10.2f} "
                   f"{meta['sigma']:<10.4f} {meta['sensing_range']:<12.3f} "
                   f"{meta['swarm_size']:<6} {meta['actual_ratio']:<15.2f}\n")
        
        f.write("="*100 + "\n\n")
        
        # Group by factors
        f.write("GROUPING BY FACTORS:\n")
        f.write("="*100 + "\n\n")
        
        # By d_des
        f.write("1. By Desired Distance (d_des):\n")
        f.write("-"*50 + "\n")
        for d_des in exp_info['d_des_values']:
            exps = [p['name'] for p in config['parameter_sets'] 
                   if p['_metadata']['d_des'] == d_des]
            f.write(f"  d_des = {d_des}: {len(exps)} experiments\n")
            for exp in exps:
                f.write(f"    - {exp}\n")
            f.write("\n")
        
        # By sensing type
        f.write("2. By Sensing Range Type:\n")
        f.write("-"*50 + "\n")
        for sensing_type in ['less', 'equal', 'greater']:
            exps = [p['name'] for p in config['parameter_sets'] 
                   if p['_metadata']['sensing_type'] == sensing_type]
            f.write(f"  {sensing_type}: {len(exps)} experiments\n")
            for exp in exps:
                f.write(f"    - {exp}\n")
            f.write("\n")
        
        # By swarm size
        f.write("3. By Swarm Size (n_s):\n")
        f.write("-"*50 + "\n")
        for n_s in exp_info['swarm_sizes']:
            exps = [p['name'] for p in config['parameter_sets'] 
                   if p['_metadata']['swarm_size'] == n_s]
            f.write(f"  n_s = {n_s}: {len(exps)} experiments\n")
            for exp in exps:
                f.write(f"    - {exp}\n")
            f.write("\n")
        
        f.write("="*100 + "\n")
    
    print(f"\n✓ Comparison summary saved to: {summary_file}")

def create_results_index(dirs, successful_experiments):
    """Create an index of all results for easy navigation."""
    
    index_file = os.path.join(dirs['summaries'], 'results_index.txt')
    
    with open(index_file, 'w') as f:
        f.write("="*100 + "\n")
        f.write("EXPERIMENT RESULTS INDEX\n")
        f.write("="*100 + "\n\n")
        
        f.write(f"Total Successful Experiments: {len(successful_experiments)}\n\n")
        
        f.write("RESULTS LOCATION:\n")
        f.write("-"*100 + "\n")
        
        for exp_name in successful_experiments:
            # Find the result directories
            result_dirs = [d for d in os.listdir(dirs['results']) 
                          if exp_name in d]
            
            if result_dirs:
                result_dir = result_dirs[0]  # Take the most recent
                result_path = os.path.join(dirs['results'], result_dir)
                
                f.write(f"\n{exp_name}:\n")
                f.write(f"  Location: {result_path}\n")
                f.write(f"  Files:\n")
                
                # List key files
                for file in os.listdir(result_path):
                    if file.endswith(('.png', '.json', '.csv', '.txt')):
                        f.write(f"    - {file}\n")
        
        f.write("\n" + "="*100 + "\n")
    
    print(f"✓ Results index saved to: {index_file}")

def main():
    """Main function to run all systematic experiments."""
    
    parser = argparse.ArgumentParser(
        description='Run systematic experiments exploring d_des, sensing_range, and swarm_size'
    )
    parser.add_argument('--trials', type=int, default=3,
                       help='Number of trials per experiment (default: 3)')
    parser.add_argument('--duration', type=int, default=200,
                       help='Duration of each trial in seconds (default: 200)')
    parser.add_argument('--start_from', type=int, default=1,
                       help='Start from experiment number (default: 1)')
    parser.add_argument('--end_at', type=int, default=None,
                       help='End at experiment number (default: all)')
    
    args = parser.parse_args()
    
    print("\n" + "="*80)
    print("SYSTEMATIC EXPERIMENT BATCH RUNNER")
    print("="*80)
    print(f"Trials per experiment: {args.trials}")
    print(f"Duration per trial: {args.duration}s")
    
    # Create output directory structure
    dirs = create_output_structure()
    print(f"\nResults will be saved to: {dirs['root']}")
    
    # Generate experiment configurations
    print("\nGenerating experiment configurations...")
    config = gen_config.generate_all_experiments()
    
    # Update with custom parameters
    config = update_config_with_custom_params(config, args.trials, args.duration)
    
    # Save configuration
    config_file = os.path.join(dirs['configs'], 'experiments_config.json')
    with open(config_file, 'w') as f:
        json.dump(config, f, indent=2)
    print(f"✓ Configuration saved to: {config_file}")
    
    # Create comparison summary
    create_comparison_summary(dirs, config)
    
    # Get list of experiments to run
    all_experiments = config['parameter_sets']
    start_idx = args.start_from - 1
    end_idx = args.end_at if args.end_at else len(all_experiments)
    experiments_to_run = all_experiments[start_idx:end_idx]
    
    total_experiments = len(experiments_to_run)
    print(f"\n{'='*80}")
    print(f"Running {total_experiments} experiments (#{args.start_from} to #{end_idx})")
    print(f"{'='*80}\n")
    
    # Run experiments
    successful_experiments = []
    failed_experiments = []
    
    start_time = datetime.now()
    
    for i, pset in enumerate(experiments_to_run, 1):
        exp_name = pset['name']
        exp_num = start_idx + i
        
        print(f"\n[{i}/{total_experiments}] Experiment #{exp_num}: {exp_name}")
        print(f"Description: {pset['description']}")
        
        success = run_single_experiment(
            config_file,
            exp_name,
            dirs['results'],
            dirs
        )
        
        if success:
            successful_experiments.append(exp_name)
        else:
            failed_experiments.append(exp_name)
        
        # Progress update
        elapsed = (datetime.now() - start_time).total_seconds()
        avg_time = elapsed / i
        remaining = (total_experiments - i) * avg_time
        
        print(f"\nProgress: {i}/{total_experiments} complete")
        print(f"Elapsed: {elapsed/60:.1f} min, Estimated remaining: {remaining/60:.1f} min")
    
    # Final summary
    end_time = datetime.now()
    total_time = (end_time - start_time).total_seconds()
    
    print("\n" + "="*80)
    print("BATCH RUN COMPLETE")
    print("="*80)
    print(f"Total time: {total_time/60:.1f} minutes ({total_time/3600:.2f} hours)")
    print(f"Successful: {len(successful_experiments)}/{total_experiments}")
    print(f"Failed: {len(failed_experiments)}/{total_experiments}")
    
    if failed_experiments:
        print("\nFailed experiments:")
        for exp in failed_experiments:
            print(f"  - {exp}")
    
    # Create results index
    if successful_experiments:
        create_results_index(dirs, successful_experiments)
    
    print(f"\nAll results saved to: {dirs['root']}")
    print(f"Summary files in: {dirs['summaries']}")
    print(f"Individual results in: {dirs['results']}")
    
    # Save final summary
    summary_data = {
        'start_time': start_time.isoformat(),
        'end_time': end_time.isoformat(),
        'total_time_seconds': total_time,
        'total_experiments': total_experiments,
        'successful': len(successful_experiments),
        'failed': len(failed_experiments),
        'successful_experiments': successful_experiments,
        'failed_experiments': failed_experiments,
        'configuration': {
            'trials_per_experiment': args.trials,
            'duration_per_trial': args.duration
        }
    }
    
    summary_file = os.path.join(dirs['summaries'], 'batch_run_summary.json')
    with open(summary_file, 'w') as f:
        json.dump(summary_data, f, indent=2)
    
    print(f"\n✓ Batch summary saved to: {summary_file}")
    print("\n" + "="*80 + "\n")

if __name__ == "__main__":
    main()