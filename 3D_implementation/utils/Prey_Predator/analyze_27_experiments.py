"""
Comprehensive analysis framework for 27 systematic experiments.

This script:
1. Loads all experiment results
2. Computes key metrics
3. Identifies optimal parameters
4. Creates comparison visualizations
5. Performs statistical analysis

Usage:
    python analyze_27_experiments.py --results_dir systematic_experiments/run_20251108_210412/results
"""

import os
import json
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import argparse
from scipy import stats
from sklearn.preprocessing import StandardScaler

# Set style
sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (15, 10)


class ExperimentAnalyzer:
    """Analyze all 27 experiments and identify optimal parameters."""
    
    def __init__(self, results_dir):
        self.results_dir = Path(results_dir)
        self.experiments = []
        self.df = None
        
    def load_all_experiments(self):
        """Load data from all experiment result folders."""
        print("Loading experiment results...")
        
        for exp_dir in self.results_dir.iterdir():
            if not exp_dir.is_dir() or not exp_dir.name.startswith('results_'):
                continue
            
            # Extract experiment name from directory
            exp_name = exp_dir.name.split('_', 1)[1].rsplit('_', 1)[0]
            
            # Load metadata
            metadata_file = list(exp_dir.glob('metadata_*.json'))
            if not metadata_file:
                print(f"  Warning: No metadata found for {exp_name}")
                continue
            
            with open(metadata_file[0], 'r') as f:
                metadata = json.load(f)
            
            # Load behavior statistics
            stats_file = list(exp_dir.glob('behavior_statistics_*.json'))
            if not stats_file:
                print(f"  Warning: No statistics found for {exp_name}")
                continue
            
            with open(stats_file[0], 'r') as f:
                behavior_stats = json.load(f)
            
            # Load trial summary
            summary_file = list(exp_dir.glob('trial_summary_*.csv'))
            if not summary_file:
                print(f"  Warning: No trial summary found for {exp_name}")
                continue
            
            trial_df = pd.read_csv(summary_file[0])
            
            # Combine all data
            exp_data = {
                'name': exp_name,
                'metadata': metadata,
                'behavior_stats': behavior_stats,
                'trial_data': trial_df,
                'result_dir': str(exp_dir)
            }
            
            self.experiments.append(exp_data)
        
        print(f"✓ Loaded {len(self.experiments)} experiments")
        
    def create_summary_dataframe(self):
        """Create a comprehensive summary DataFrame."""
        print("\nCreating summary DataFrame...")
        
        rows = []
        
        for exp in self.experiments:
            metadata = exp['metadata']
            behavior_stats = exp['behavior_stats']
            trial_data = exp['trial_data']
            
            # Extract parameters from flocking_parameters
            flocking = metadata['flocking_parameters']
            
            row = {
                # Experiment identification
                'experiment': exp['name'],
                
                # Design parameters (factors)
                'd_des': flocking['desired_distance'],
                'sigma': flocking['desired_distance'] / np.sqrt(2),
                'sensing_range': flocking['sensing_range_own_swarm'],
                'swarm_size': metadata['n_predators'],
                
                # Derived metrics
                'sensing_ratio': flocking['sensing_range_own_swarm'] / flocking['desired_distance'],
                
                # Behavior counts (windowed - last 150s)
                'following_mean': behavior_stats['following']['mean'],
                'following_std': behavior_stats['following']['std'],
                'engulfing_mean': behavior_stats['engulfing']['mean'],
                'engulfing_std': behavior_stats['engulfing']['std'],
                'separating_mean': behavior_stats['separating']['mean'],
                'separating_std': behavior_stats['separating']['std'],
                
                # Distance metrics (averaged across trials)
                'avg_distance_mean': trial_data['avg_distance'].mean(),
                'avg_distance_std': trial_data['avg_distance'].std(),
                'min_distance_mean': trial_data['min_distance'].mean(),
                'max_distance_mean': trial_data['max_distance'].mean(),
                'std_distance_mean': trial_data['std_distance'].mean(),
                
                # Derived metrics
                'total_behaviors': (behavior_stats['following']['mean'] + 
                                   behavior_stats['engulfing']['mean'] + 
                                   behavior_stats['separating']['mean']),
                
                'interaction_index': (behavior_stats['following']['mean'] + 
                                     behavior_stats['engulfing']['mean']) / 
                                    (behavior_stats['separating']['mean'] + 1),  # +1 to avoid div by zero
                
                'pursuit_success': behavior_stats['engulfing']['mean'] / 
                                  (behavior_stats['following']['mean'] + 
                                   behavior_stats['engulfing']['mean'] + 0.001),  # Proportion of close interactions
                
                'dynamic_range': trial_data['max_distance'].mean() - trial_data['min_distance'].mean(),
                
                # Consistency metrics
                'behavior_consistency': 1 / (behavior_stats['following']['std'] + 
                                            behavior_stats['engulfing']['std'] + 
                                            behavior_stats['separating']['std'] + 1),
                
                'distance_stability': 1 / (trial_data['std_distance'].mean() + 0.1)
            }
            
            rows.append(row)
        
        self.df = pd.DataFrame(rows)
        
        # Add categorical variables for easier analysis
        self.df['d_des_cat'] = pd.Categorical(self.df['d_des'])
        self.df['swarm_size_cat'] = pd.Categorical(self.df['swarm_size'])
        
        # Categorize sensing type
        self.df['sensing_type'] = self.df['sensing_ratio'].apply(
            lambda x: 'less' if x < 1.6 else ('equal' if x < 2.0 else 'greater')
        )
        
        print(f"✓ Created DataFrame with {len(self.df)} experiments and {len(self.df.columns)} metrics")
        
        return self.df
    
    def print_summary_statistics(self):
        """Print overall summary statistics."""
        print("\n" + "="*80)
        print("SUMMARY STATISTICS ACROSS ALL 27 EXPERIMENTS")
        print("="*80)
        
        print("\n1. BEHAVIOR COUNTS (Mean across experiments)")
        print("-"*50)
        print(f"Following:   {self.df['following_mean'].mean():.2f} ± {self.df['following_mean'].std():.2f}")
        print(f"Engulfing:   {self.df['engulfing_mean'].mean():.2f} ± {self.df['engulfing_mean'].std():.2f}")
        print(f"Separating:  {self.df['separating_mean'].mean():.2f} ± {self.df['separating_mean'].std():.2f}")
        
        print("\n2. DISTANCE METRICS (Mean across experiments)")
        print("-"*50)
        print(f"Average distance:  {self.df['avg_distance_mean'].mean():.2f}m ± {self.df['avg_distance_mean'].std():.2f}m")
        print(f"Min distance:      {self.df['min_distance_mean'].mean():.2f}m ± {self.df['min_distance_mean'].std():.2f}m")
        print(f"Max distance:      {self.df['max_distance_mean'].mean():.2f}m ± {self.df['max_distance_mean'].std():.2f}m")
        
        print("\n3. INTERACTION QUALITY")
        print("-"*50)
        print(f"Interaction index: {self.df['interaction_index'].mean():.2f} ± {self.df['interaction_index'].std():.2f}")
        print(f"Pursuit success:   {self.df['pursuit_success'].mean():.2f} ± {self.df['pursuit_success'].std():.2f}")
        
    def identify_optimal_parameters(self, objective='balanced_interaction'):
        """
        Identify optimal parameters based on different objectives.
        
        Objectives:
        - 'balanced_interaction': Mix of following and engulfing
        - 'max_pursuit': Maximum predator success (engulfing)
        - 'max_evasion': Maximum prey escape (separating)
        - 'dynamic': High variability and dynamic behavior
        - 'stable': Consistent, predictable behavior
        """
        print(f"\n{'='*80}")
        print(f"IDENTIFYING OPTIMAL PARAMETERS FOR: {objective.upper()}")
        print(f"{'='*80}\n")
        
        if objective == 'balanced_interaction':
            # Want both following and engulfing, minimal separating
            self.df['objective_score'] = (
                self.df['following_mean'] * 0.4 +
                self.df['engulfing_mean'] * 0.4 +
                self.df['interaction_index'] * 0.2 -
                self.df['separating_mean'] * 0.5
            )
            
        elif objective == 'max_pursuit':
            # Maximize engulfing, minimize escape
            self.df['objective_score'] = (
                self.df['engulfing_mean'] * 0.6 +
                self.df['pursuit_success'] * 0.3 -
                self.df['avg_distance_mean'] * 0.1
            )
            
        elif objective == 'max_evasion':
            # Maximize prey escape capability
            self.df['objective_score'] = (
                self.df['separating_mean'] * 0.5 +
                self.df['avg_distance_mean'] * 0.3 +
                self.df['dynamic_range'] * 0.2
            )
            
        elif objective == 'dynamic':
            # Maximize behavioral diversity and variability
            self.df['objective_score'] = (
                self.df['total_behaviors'] * 0.4 +
                self.df['dynamic_range'] * 0.3 +
                self.df['std_distance_mean'] * 0.3
            )
            
        elif objective == 'stable':
            # Minimize variability, maximize consistency
            self.df['objective_score'] = (
                self.df['behavior_consistency'] * 0.5 +
                self.df['distance_stability'] * 0.3 +
                (1 / (self.df['following_std'] + self.df['engulfing_std'] + 1)) * 0.2
            )
        
        # Rank experiments
        ranked = self.df.sort_values('objective_score', ascending=False)
        
        print("TOP 5 PARAMETER COMBINATIONS:")
        print("-"*80)
        print(f"{'Rank':<6} {'Experiment':<20} {'d_des':<8} {'Sensing':<12} {'n':<6} {'Score':<10}")
        print("-"*80)
        
        for i, (idx, row) in enumerate(ranked.head(5).iterrows(), 1):
            print(f"{i:<6} {row['experiment']:<20} {row['d_des']:<8.1f} "
                  f"{row['sensing_range']:<12.2f} {row['swarm_size']:<6.0f} {row['objective_score']:<10.3f}")
        
        print("\n" + "="*80)
        
        # Detailed analysis of top performer
        best = ranked.iloc[0]
        print(f"\nBEST CONFIGURATION: {best['experiment']}")
        print("="*80)
        print(f"Parameters:")
        print(f"  d_des: {best['d_des']:.2f}m")
        print(f"  σ: {best['sigma']:.4f}m")
        print(f"  Sensing range: {best['sensing_range']:.2f}m ({best['sensing_ratio']:.2f}×d_des)")
        print(f"  Swarm size: {best['swarm_size']:.0f} agents")
        print(f"\nBehavior Metrics:")
        print(f"  Following: {best['following_mean']:.2f} ± {best['following_std']:.2f}")
        print(f"  Engulfing: {best['engulfing_mean']:.2f} ± {best['engulfing_std']:.2f}")
        print(f"  Separating: {best['separating_mean']:.2f} ± {best['separating_std']:.2f}")
        print(f"\nDistance Metrics:")
        print(f"  Average: {best['avg_distance_mean']:.2f}m ± {best['avg_distance_std']:.2f}m")
        print(f"  Range: {best['min_distance_mean']:.2f}m - {best['max_distance_mean']:.2f}m")
        print(f"\nQuality Indices:")
        print(f"  Interaction index: {best['interaction_index']:.2f}")
        print(f"  Pursuit success: {best['pursuit_success']:.2f}")
        print(f"  Dynamic range: {best['dynamic_range']:.2f}m")
        print("="*80)
        
        return ranked
    
    def create_factor_analysis_plots(self, output_dir):
        """Create comprehensive factor analysis visualizations."""
        print(f"\nCreating factor analysis plots...")
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # 1. Main effects plot - How each factor affects key metrics
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('Main Effects: How Each Factor Affects Behavior', fontsize=16, fontweight='bold')
        
        metrics = [
            ('following_mean', 'Following Count'),
            ('engulfing_mean', 'Engulfing Count'),
            ('separating_mean', 'Separating Count'),
            ('avg_distance_mean', 'Average Distance (m)'),
            ('interaction_index', 'Interaction Index'),
            ('pursuit_success', 'Pursuit Success')
        ]
        
        for ax, (metric, title) in zip(axes.flat, metrics):
            # Effect of d_des
            d_des_effect = self.df.groupby('d_des')[metric].agg(['mean', 'std'])
            ax.errorbar(d_des_effect.index, d_des_effect['mean'], yerr=d_des_effect['std'],
                       marker='o', linewidth=2, capsize=5, label='d_des effect', color='blue')
            
            # Effect of sensing (on secondary axis)
            ax2 = ax.twinx()
            sensing_effect = self.df.groupby('sensing_type')[metric].mean()
            sensing_order = ['less', 'equal', 'greater']
            sensing_values = [sensing_effect.get(s, 0) for s in sensing_order]
            ax2.plot(range(3), sensing_values, marker='s', linewidth=2, 
                    label='Sensing effect', color='red', linestyle='--')
            
            ax.set_xlabel('d_des (m)', fontsize=10)
            ax.set_ylabel(title, fontsize=10, color='blue')
            ax2.set_ylabel('Sensing effect', fontsize=10, color='red')
            ax.tick_params(axis='y', labelcolor='blue')
            ax2.tick_params(axis='y', labelcolor='red')
            ax.grid(alpha=0.3)
            ax.set_title(title, fontsize=12, fontweight='bold')
        
        plt.tight_layout()
        plt.savefig(output_dir / 'main_effects_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
        print(f"  ✓ Saved: main_effects_analysis.png")
        
        # 2. Heatmaps for each swarm size
        for n in sorted(self.df['swarm_size'].unique()):
            fig, axes = plt.subplots(1, 3, figsize=(18, 5))
            fig.suptitle(f'Behavior Heatmaps - Swarm Size n={int(n)}', 
                        fontsize=14, fontweight='bold')
            
            subset = self.df[self.df['swarm_size'] == n]
            
            metrics_to_plot = [
                ('following_mean', 'Following', 'Blues'),
                ('engulfing_mean', 'Engulfing', 'Reds'),
                ('separating_mean', 'Separating', 'Greens')
            ]
            
            for ax, (metric, title, cmap) in zip(axes, metrics_to_plot):
                pivot = subset.pivot_table(
                    values=metric,
                    index='d_des',
                    columns='sensing_type',
                    aggfunc='mean'
                )
                pivot = pivot[['less', 'equal', 'greater']]  # Ensure order
                
                sns.heatmap(pivot, annot=True, fmt='.2f', cmap=cmap, ax=ax,
                           cbar_kws={'label': 'Count'}, vmin=0)
                ax.set_title(title, fontsize=12, fontweight='bold')
                ax.set_xlabel('Sensing Type', fontsize=10)
                ax.set_ylabel('d_des (m)', fontsize=10)
            
            plt.tight_layout()
            plt.savefig(output_dir / f'heatmap_n{int(n)}.png', dpi=300, bbox_inches='tight')
            plt.close()
            print(f"  ✓ Saved: heatmap_n{int(n)}.png")
        
        # 3. Interaction plots
        fig, axes = plt.subplots(2, 2, figsize=(14, 12))
        fig.suptitle('Interaction Effects Between Factors', fontsize=16, fontweight='bold')
        
        # d_des × sensing_type interaction (for engulfing)
        ax = axes[0, 0]
        for sensing in ['less', 'equal', 'greater']:
            data = self.df[self.df['sensing_type'] == sensing]
            grouped = data.groupby('d_des')['engulfing_mean'].mean()
            ax.plot(grouped.index, grouped.values, marker='o', linewidth=2, label=sensing)
        ax.set_xlabel('d_des (m)', fontsize=11)
        ax.set_ylabel('Engulfing Count', fontsize=11)
        ax.set_title('d_des × Sensing Interaction', fontsize=12, fontweight='bold')
        ax.legend(title='Sensing')
        ax.grid(alpha=0.3)
        
        # d_des × swarm_size interaction (for interaction index)
        ax = axes[0, 1]
        for n in sorted(self.df['swarm_size'].unique()):
            data = self.df[self.df['swarm_size'] == n]
            grouped = data.groupby('d_des')['interaction_index'].mean()
            ax.plot(grouped.index, grouped.values, marker='o', linewidth=2, label=f'n={int(n)}')
        ax.set_xlabel('d_des (m)', fontsize=11)
        ax.set_ylabel('Interaction Index', fontsize=11)
        ax.set_title('d_des × Swarm Size Interaction', fontsize=12, fontweight='bold')
        ax.legend(title='Swarm Size')
        ax.grid(alpha=0.3)
        
        # sensing × swarm_size interaction (for pursuit success)
        ax = axes[1, 0]
        for n in sorted(self.df['swarm_size'].unique()):
            data = self.df[self.df['swarm_size'] == n]
            grouped = data.groupby('sensing_type')['pursuit_success'].mean()
            sensing_order = ['less', 'equal', 'greater']
            values = [grouped.get(s, 0) for s in sensing_order]
            ax.plot(range(3), values, marker='o', linewidth=2, label=f'n={int(n)}')
        ax.set_xticks(range(3))
        ax.set_xticklabels(sensing_order)
        ax.set_xlabel('Sensing Type', fontsize=11)
        ax.set_ylabel('Pursuit Success', fontsize=11)
        ax.set_title('Sensing × Swarm Size Interaction', fontsize=12, fontweight='bold')
        ax.legend(title='Swarm Size')
        ax.grid(alpha=0.3)
        
        # Average distance comparison
        ax = axes[1, 1]
        box_data = []
        labels = []
        for d_des in sorted(self.df['d_des'].unique()):
            data = self.df[self.df['d_des'] == d_des]['avg_distance_mean']
            box_data.append(data)
            labels.append(f'd={d_des}')
        ax.boxplot(box_data, labels=labels)
        ax.set_xlabel('d_des Configuration', fontsize=11)
        ax.set_ylabel('Average Distance (m)', fontsize=11)
        ax.set_title('Distance Distribution by d_des', fontsize=12, fontweight='bold')
        ax.grid(alpha=0.3, axis='y')
        
        plt.tight_layout()
        plt.savefig(output_dir / 'interaction_effects.png', dpi=300, bbox_inches='tight')
        plt.close()
        print(f"  ✓ Saved: interaction_effects.png")
        
        # 4. 3D parameter space visualization
        fig = plt.figure(figsize=(16, 5))
        
        # Engulfing in 3D parameter space
        ax1 = fig.add_subplot(131, projection='3d')
        scatter = ax1.scatter(self.df['d_des'], self.df['sensing_range'], self.df['swarm_size'],
                            c=self.df['engulfing_mean'], cmap='Reds', s=200, alpha=0.6)
        ax1.set_xlabel('d_des (m)', fontsize=10)
        ax1.set_ylabel('Sensing Range (m)', fontsize=10)
        ax1.set_zlabel('Swarm Size', fontsize=10)
        ax1.set_title('Engulfing Count', fontsize=12, fontweight='bold')
        plt.colorbar(scatter, ax=ax1, shrink=0.5)
        
        # Interaction index in 3D parameter space
        ax2 = fig.add_subplot(132, projection='3d')
        scatter = ax2.scatter(self.df['d_des'], self.df['sensing_range'], self.df['swarm_size'],
                            c=self.df['interaction_index'], cmap='viridis', s=200, alpha=0.6)
        ax2.set_xlabel('d_des (m)', fontsize=10)
        ax2.set_ylabel('Sensing Range (m)', fontsize=10)
        ax2.set_zlabel('Swarm Size', fontsize=10)
        ax2.set_title('Interaction Index', fontsize=12, fontweight='bold')
        plt.colorbar(scatter, ax=ax2, shrink=0.5)
        
        # Average distance in 3D parameter space
        ax3 = fig.add_subplot(133, projection='3d')
        scatter = ax3.scatter(self.df['d_des'], self.df['sensing_range'], self.df['swarm_size'],
                            c=self.df['avg_distance_mean'], cmap='coolwarm', s=200, alpha=0.6)
        ax3.set_xlabel('d_des (m)', fontsize=10)
        ax3.set_ylabel('Sensing Range (m)', fontsize=10)
        ax3.set_zlabel('Swarm Size', fontsize=10)
        ax3.set_title('Average Distance', fontsize=12, fontweight='bold')
        plt.colorbar(scatter, ax=ax3, shrink=0.5)
        
        plt.tight_layout()
        plt.savefig(output_dir / 'parameter_space_3d.png', dpi=300, bbox_inches='tight')
        plt.close()
        print(f"  ✓ Saved: parameter_space_3d.png")
    
    def perform_statistical_tests(self):
        """Perform ANOVA and post-hoc tests."""
        print("\n" + "="*80)
        print("STATISTICAL ANALYSIS")
        print("="*80)
        
        # One-way ANOVA for each factor
        print("\n1. ONE-WAY ANOVA (Effect of each factor)")
        print("-"*80)
        
        metrics = ['following_mean', 'engulfing_mean', 'separating_mean', 'interaction_index']
        factors = ['d_des', 'sensing_type', 'swarm_size']
        
        for metric in metrics:
            print(f"\n{metric.upper().replace('_', ' ')}:")
            for factor in factors:
                groups = [group[metric].values for name, group in self.df.groupby(factor)]
                f_stat, p_value = stats.f_oneway(*groups)
                sig = "***" if p_value < 0.001 else ("**" if p_value < 0.01 else ("*" if p_value < 0.05 else "ns"))
                print(f"  {factor}: F={f_stat:.3f}, p={p_value:.4f} {sig}")
        
        print("\n" + "="*80)
        print("Significance: *** p<0.001, ** p<0.01, * p<0.05, ns not significant")
        print("="*80)
    
    def export_summary_tables(self, output_dir):
        """Export summary tables as CSV."""
        print(f"\nExporting summary tables...")
        output_dir = Path(output_dir)
        
        # Full summary
        self.df.to_csv(output_dir / 'full_summary.csv', index=False)
        print(f"  ✓ Saved: full_summary.csv")
        
        # Factor means
        for factor in ['d_des', 'sensing_type', 'swarm_size']:
            grouped = self.df.groupby(factor).agg({
                'following_mean': ['mean', 'std'],
                'engulfing_mean': ['mean', 'std'],
                'separating_mean': ['mean', 'std'],
                'avg_distance_mean': ['mean', 'std'],
                'interaction_index': ['mean', 'std']
            })
            grouped.to_csv(output_dir / f'summary_by_{factor}.csv')
            print(f"  ✓ Saved: summary_by_{factor}.csv")
        
        # Top performers for each objective
        objectives = ['balanced_interaction', 'max_pursuit', 'max_evasion', 'dynamic', 'stable']
        top_configs = []
        
        for obj in objectives:
            ranked = self.identify_optimal_parameters(objective=obj)
            best = ranked.iloc[0]
            top_configs.append({
                'objective': obj,
                'experiment': best['experiment'],
                'd_des': best['d_des'],
                'sensing_range': best['sensing_range'],
                'swarm_size': best['swarm_size'],
                'score': best['objective_score']
            })
        
        pd.DataFrame(top_configs).to_csv(output_dir / 'optimal_configs_by_objective.csv', index=False)
        print(f"  ✓ Saved: optimal_configs_by_objective.csv")


def main():
    parser = argparse.ArgumentParser(description='Analyze 27 systematic experiments')
    parser.add_argument('--results_dir', type=str, required=True,
                       help='Path to results directory')
    parser.add_argument('--output_dir', type=str, default='analysis_output',
                       help='Directory for analysis outputs')
    parser.add_argument('--objective', type=str, default='balanced_interaction',
                       choices=['balanced_interaction', 'max_pursuit', 'max_evasion', 'dynamic', 'stable'],
                       help='Optimization objective')
    
    args = parser.parse_args()
    
    # Create analyzer
    analyzer = ExperimentAnalyzer(args.results_dir)
    
    # Load all experiments
    analyzer.load_all_experiments()
    
    # Create summary DataFrame
    analyzer.create_summary_dataframe()
    
    # Print summary statistics
    analyzer.print_summary_statistics()
    
    # Identify optimal parameters
    analyzer.identify_optimal_parameters(objective=args.objective)
    
    # Create visualizations
    analyzer.create_factor_analysis_plots(args.output_dir)
    
    # Statistical tests
    analyzer.perform_statistical_tests()
    
    # Export tables
    analyzer.export_summary_tables(args.output_dir)
    
    print("\n" + "="*80)
    print("ANALYSIS COMPLETE!")
    print("="*80)
    print(f"All outputs saved to: {args.output_dir}")
    print("\nGenerated files:")
    print("  - main_effects_analysis.png")
    print("  - heatmap_n*.png (one per swarm size)")
    print("  - interaction_effects.png")
    print("  - parameter_space_3d.png")
    print("  - full_summary.csv")
    print("  - summary_by_*.csv (factor summaries)")
    print("  - optimal_configs_by_objective.csv")
    print("="*80 + "\n")


if __name__ == "__main__":
    main()