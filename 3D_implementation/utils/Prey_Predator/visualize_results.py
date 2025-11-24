"""
Standalone visualization script for predator-prey experiment results.

This script can visualize results from saved experiment data without re-running simulations.

Usage:
    python visualize_results.py --results_dir experiment_results/results_baseline_03-10-2025_14-30
    python visualize_results.py --results_dir experiment_results/results_baseline_03-10-2025_14-30 --output_format pdf
    python visualize_results.py --results_dir experiment_results/results_baseline_03-10-2025_14-30 --summary_only
"""

import os
import sys
import argparse
import json
import pickle
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

class ResultsVisualizer:
    """Visualize predator-prey experiment results from saved data."""
    
    def __init__(self, results_dir):
        self.results_dir = results_dir
        self.load_data()
    
    def load_data(self):
        """Load all necessary data from the results directory."""
        if not os.path.isdir(self.results_dir):
            raise NotADirectoryError(f"Results directory not found: {self.results_dir}")
        
        plot_files = [f for f in os.listdir(self.results_dir) 
                      if f.startswith('plot_data_') and f.endswith('.pkl')]
        
        if not plot_files:
            raise FileNotFoundError(f"No plot data file found in {self.results_dir}")
        
        # pick the latest by mtime if multiple exist
        plot_files_full = [os.path.join(self.results_dir, f) for f in plot_files]
        plot_file = max(plot_files_full, key=os.path.getmtime)
        print(f"Loading data from: {plot_file}")
        
        with open(plot_file, 'rb') as f:
            data = pickle.load(f)
        
        self.stats = data['stats']
        self.all_results = data['all_results']
        self.config = data['config']
        self.parameter_set_name = data.get('parameter_set_name', 'parameters')
        
        print(f"Loaded data for parameter set: {self.parameter_set_name}")
        print(f"Number of trials: {len(self.all_results)}")
    
    def create_summary_plot(self, save_format='png'):
        """Create the 3-panel summary visualization."""
        control_freq = self.config['simulation']['default_control_freq_hz']
        thresholds = self.all_results[0]['thresholds']
        analysis_window = self.config.get('analysis_window_sec', 150)
        
        fig, axes = plt.subplots(1, 3, figsize=(20, 6))
        title = f'Predator-Prey Behavior Analysis - {self.parameter_set_name} (Last {analysis_window}s)'
        fig.suptitle(title, fontsize=16, fontweight='bold')
        
        # 1. Bar chart of mean behavior counts
        behaviors = list(self.stats.keys())
        means = [self.stats[b]['mean'] for b in behaviors]
        stds = [self.stats[b]['std'] for b in behaviors]
        
        ax1 = axes[0]
        bars = ax1.bar(behaviors, means, yerr=stds, capsize=5, 
                       color=['skyblue', 'lightcoral', 'lightgreen'])
        ax1.set_title(f'Mean Behavior Counts per Trial\n(Last {analysis_window}s)')
        ax1.set_ylabel('Count')
        ax1.grid(axis='y', alpha=0.3)
        
        for bar, mean, std in zip(bars, means, stds):
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(std, 0) + 0.1,
                     f'{mean:.1f}', ha='center', va='bottom')
        
        # 2. Distance Evolution - All Trials
        ax2 = axes[1]
        colors = plt.cm.tab10(np.linspace(0, 1, max(len(self.all_results), 2)))
        
        for i, result in enumerate(self.all_results):
            distances = np.asarray(result['distances'])
            time_steps = np.arange(len(distances)) / control_freq
            ax2.plot(time_steps, distances, alpha=0.6, linewidth=1, 
                     color=colors[i % len(colors)], label=f'Trial {i+1}')
        
        # Threshold lines
        ax2.axhline(y=thresholds['follow'], color='orange', linestyle='--', linewidth=2,
                    label=f'Follow ({thresholds["follow"]}m)')
        ax2.axhline(y=thresholds['engulf'], color='red', linestyle='--', linewidth=2,
                    label=f'Engulf ({thresholds["engulf"]}m)')
        ax2.axhline(y=thresholds['separation'], color='green', linestyle='--', linewidth=2,
                    label=f'Separation ({thresholds["separation"]}m)')
        
        # Highlight analysis window
        total_time = len(self.all_results[0]['distances']) / control_freq
        analysis_start_time = max(0.0, total_time - analysis_window)
        ax2.axvspan(analysis_start_time, total_time, alpha=0.1, color='gray', 
                    label=f'Analysis Window ({analysis_window}s)')
        
        ax2.set_title('Distance Evolution - All Trials')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Min Distance (m)')
        ax2.grid(alpha=0.3)
        
        handles, labels = ax2.get_legend_handles_labels()
        # Keep only threshold + window entries to avoid crowded legends
        if len(handles) >= 4:
            ax2.legend(handles[-4:], labels[-4:], loc='upper right', fontsize=8)
        else:
            ax2.legend(loc='upper right', fontsize=8)
        
        # 3. Distance Evolution - Mean ± Std
        ax3 = axes[2]
        min_length = min(len(result['distances']) for result in self.all_results)
        aligned_distances = np.array([np.asarray(result['distances'])[:min_length] 
                                      for result in self.all_results])
        
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
        analysis_start_time = max(0.0, (min_length / control_freq) - analysis_window)
        ax3.axvspan(analysis_start_time, min_length / control_freq, alpha=0.1, color='gray',
                    label=f'Analysis Window ({analysis_window}s)')
        
        ax3.set_title('Distance Evolution - Mean ± Std')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Min Distance (m)')
        ax3.legend(fontsize=8)
        ax3.grid(alpha=0.3)
        
        fig.tight_layout(rect=[0, 0.02, 1, 0.95])
        filename = f'behavior_analysis_{self.parameter_set_name}.{save_format}'
        filepath = os.path.join(self.results_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved: {filepath}")
    
    def create_individual_trials_plot(self, save_format='png'):
        """Create individual distance plots for each trial."""
        control_freq = self.config['simulation']['default_control_freq_hz']
        thresholds = self.all_results[0]['thresholds']
        
        n_trials = len(self.all_results)
        n_cols = 3
        n_rows = (n_trials + n_cols - 1) // n_cols
        
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(15, 4 * n_rows))
        fig.suptitle(f'Individual Distance Evolution - {self.parameter_set_name}', 
                     fontsize=16)
        
        # Normalize axes to a flat list
        if n_rows == 1 and n_cols == 1:
            axes = [axes]
        elif n_rows == 1 or n_cols == 1:
            axes = np.ravel(axes)
        else:
            axes = axes.flatten()
        
        for i, result in enumerate(self.all_results):
            ax = axes[i]
            distances = np.asarray(result['distances'])
            time_steps = np.arange(len(distances)) / control_freq
            
            ax.plot(time_steps, distances, 'b-', alpha=0.8, linewidth=1.5)
            
            # Thresholds
            ax.axhline(y=thresholds['follow'], color='orange', linestyle='--', alpha=0.8)
            ax.axhline(y=thresholds['engulf'], color='red', linestyle='--', alpha=0.8)
            ax.axhline(y=thresholds['separation'], color='green', linestyle='--', alpha=0.8)
            
            ax.set_title(f'Trial {i+1}')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Min Distance (m)')
            ax.grid(alpha=0.3)
            
            # Stats text
            behavior_counts = result['windowed_behavior_counts']
            stats_text = (f"F:{behavior_counts['following']} "
                          f"E:{behavior_counts['engulfing']} "
                          f"S:{behavior_counts['separating']}")
            ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
                    verticalalignment='top', fontsize=8, 
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        # Hide unused subplots correctly
        for j in range(n_trials, len(axes)):
            axes[j].set_visible(False)
        
        fig.tight_layout(rect=[0, 0.03, 1, 0.95])
        filename = f'individual_trials_{self.parameter_set_name}.{save_format}'
        filepath = os.path.join(self.results_dir, filename)
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        plt.close(fig)
        print(f"Saved: {filepath}")
    
    def print_text_summary(self):
        """Print a concise text summary to stdout."""
        print("\n=== Text Summary ===")
        print(f"Parameter set: {self.parameter_set_name}")
        for behavior, s in self.stats.items():
            mean = s.get('mean', np.nan)
            std = s.get('std', np.nan)
            print(f"- {behavior}: mean={mean:.2f}, std={std:.2f}")
        print(f"Trials: {len(self.all_results)}")

def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize predator-prey experiment results from saved pickle files."
    )
    parser.add_argument(
        "--results_dir",
        required=True,
        help="Directory containing plot_data_*.pkl created by the experiment runner."
    )
    parser.add_argument(
        "--output_format",
        default="png",
        choices=["png", "pdf", "svg"],
        help="Image format for saved figures."
    )
    parser.add_argument(
        "--summary_only",
        action="store_true",
        help="Only print text summary without generating plots."
    )
    parser.add_argument(
        "--no_individual",
        action="store_true",
        help="Skip saving the per-trial grid figure."
    )
    return parser.parse_args()

def main():
    args = parse_args()
    vis = ResultsVisualizer(args.results_dir)
    
    # Always print summary
    vis.print_text_summary()
    
    if args.summary_only:
        return
    
    # Create plots
    vis.create_summary_plot(save_format=args.output_format)
    if not args.no_individual:
        vis.create_individual_trials_plot(save_format=args.output_format)

if __name__ == "__main__":
    main()
