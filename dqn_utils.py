"""
Advanced DQN utilities and analysis tools
"""

import torch
import numpy as np
import pickle
from pathlib import Path
import sys
from datetime import datetime

# Matplotlib import deferred
matplotlib_available = False
try:
    import matplotlib.pyplot as plt
    matplotlib_available = True
except ImportError:
    pass

# Add MPC to path
sys.path.insert(0, str(Path(__file__).parent / "MPC"))

from MPC.map_simulation import MapSimulation
from MPC.map import random_map_generater
from train_dqn import DQNAgent, train, test


class DQNExperimentTracker:
    """Track and compare multiple DQN training runs"""
    
    def __init__(self, experiment_name="dqn_experiment"):
        self.experiment_name = experiment_name
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.experiment_dir = Path(f"experiments/{experiment_name}_{self.timestamp}")
        self.experiment_dir.mkdir(parents=True, exist_ok=True)
        self.results = {}
        
        print(f"Experiment directory: {self.experiment_dir}")
    
    def run_training(self, config_name, config_dict, episodes=100):
        """Run training with specific configuration"""
        print(f"\n{'='*60}")
        print(f"Running: {config_name}")
        print(f"{'='*60}\n")
        
        env = MapSimulation(random_map_generater(num_cleaners=2, num_windows=5), real_time=False)
        state_dim = len(env.get_dqn_state())
        action_dim = len(env.new_build_drone_actions())
        
        agent = DQNAgent(state_dim, action_dim, **config_dict)
        rewards, lengths = train(agent, env, episodes=episodes, max_steps=150)
        
        self.results[config_name] = {
            'config': config_dict,
            'rewards': rewards,
            'lengths': lengths,
            'agent': agent
        }
        
        # Save agent
        model_path = self.experiment_dir / f"{config_name}_model.pt"
        agent.save(str(model_path))
        
        return agent, rewards, lengths
    
    def compare_configurations(self):
        """Compare all trained configurations"""
        if not self.results:
            print("No results to compare. Run training first.")
            return
        
        if not matplotlib_available:
            print("Warning: matplotlib not available, skipping visualization")
            return
        
        fig, axes = plt.subplots(len(self.results), 2, figsize=(14, 5*len(self.results)))
        if len(self.results) == 1:
            axes = axes.reshape(1, 2)
        
        for idx, (config_name, data) in enumerate(self.results.items()):
            rewards = data['rewards']
            lengths = data['lengths']
            
            # Rewards
            axes[idx, 0].plot(rewards, alpha=0.6)
            window = 10
            if len(rewards) > window:
                moving_avg = np.convolve(rewards, np.ones(window)/window, mode='valid')
                axes[idx, 0].plot(range(window-1, len(rewards)), moving_avg, linewidth=2)
            axes[idx, 0].set_title(f'{config_name} - Rewards')
            axes[idx, 0].set_ylabel('Reward')
            axes[idx, 0].grid(True, alpha=0.3)
            
            # Lengths
            axes[idx, 1].plot(lengths, alpha=0.6)
            if len(lengths) > window:
                moving_avg = np.convolve(lengths, np.ones(window)/window, mode='valid')
                axes[idx, 1].plot(range(window-1, len(lengths)), moving_avg, linewidth=2)
            axes[idx, 1].set_title(f'{config_name} - Episode Length')
            axes[idx, 1].set_ylabel('Steps')
            axes[idx, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        save_path = self.experiment_dir / "comparison.png"
        plt.savefig(save_path, dpi=100, bbox_inches='tight')
        print(f"\nComparison saved to {save_path}")
        plt.show()
    
    def print_summary(self):
        """Print summary statistics"""
        print(f"\n{'='*60}")
        print("Experiment Summary")
        print(f"{'='*60}\n")
        
        for config_name, data in self.results.items():
            rewards = data['rewards']
            print(f"{config_name}:")
            print(f"  Mean Reward: {np.mean(rewards):.1f} ± {np.std(rewards):.1f}")
            print(f"  Max Reward: {np.max(rewards):.1f}")
            print(f"  Min Reward: {np.min(rewards):.1f}")
            print()


def analyze_agent_behavior(model_path, num_episodes=5, max_steps=200):
    """Analyze and visualize agent behavior"""
    
    env = MapSimulation(random_map_generater(num_cleaners=2, num_windows=5), real_time=False)
    state_dim = len(env.get_dqn_state())
    action_dim = len(env.new_build_drone_actions())
    
    agent = DQNAgent(state_dim, action_dim)
    agent.load(model_path)
    
    print(f"\n{'='*60}")
    print("Agent Behavior Analysis")
    print(f"{'='*60}\n")
    
    action_counts = np.zeros(action_dim)
    state_values = []
    rewards_sequence = []
    
    for ep in range(num_episodes):
        env = MapSimulation(random_map_generater(num_cleaners=2, num_windows=5), real_time=False)
        state = env.get_dqn_state()
        
        episode_rewards = []
        
        for t in range(max_steps):
            # Get Q-values
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(agent.device)
            with torch.no_grad():
                q_values = agent.model(state_tensor).cpu().numpy()[0]
            
            action = agent.act(state, training=False)
            next_state, reward, done, _ = env.rl_step(action)
            
            action_counts[action] += 1
            state_values.append(np.max(q_values))
            episode_rewards.append(reward)
            
            state = next_state
            
            if done:
                break
        
        rewards_sequence.extend(episode_rewards)
    
    if not matplotlib_available:
        print("Warning: matplotlib not available, skipping visualization")
    else:
        # Create analysis plots
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # Action distribution
        axes[0, 0].bar(range(action_dim), action_counts)
        axes[0, 0].set_xlabel('Action')
        axes[0, 0].set_ylabel('Count')
        axes[0, 0].set_title('Action Distribution')
        axes[0, 0].grid(True, alpha=0.3, axis='y')
        
        # Max Q-values
        axes[0, 1].plot(state_values, alpha=0.7)
        axes[0, 1].set_xlabel('Step')
        axes[0, 1].set_ylabel('Max Q-Value')
        axes[0, 1].set_title('Q-Value Evolution')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Reward sequence
        axes[1, 0].plot(rewards_sequence, alpha=0.7)
        axes[1, 0].set_xlabel('Step')
        axes[1, 0].set_ylabel('Reward')
        axes[1, 0].set_title('Reward Sequence')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Q-value distribution
        axes[1, 1].hist(state_values, bins=50, alpha=0.7, edgecolor='black')
        axes[1, 1].set_xlabel('Max Q-Value')
        axes[1, 1].set_ylabel('Frequency')
        axes[1, 1].set_title('Q-Value Distribution')
        axes[1, 1].grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        plt.savefig("agent_behavior_analysis.png", dpi=100, bbox_inches='tight')
        print("Analysis saved to agent_behavior_analysis.png")
    
    print(f"\nAction Statistics:")
    for i, count in enumerate(action_counts):
        print(f"  Action {i}: {count:.0f} times ({count/np.sum(action_counts)*100:.1f}%)")
    
    print(f"\nQ-Value Statistics:")
    print(f"  Mean: {np.mean(state_values):.4f}")
    print(f"  Std: {np.std(state_values):.4f}")
    print(f"  Min: {np.min(state_values):.4f}")
    print(f"  Max: {np.max(state_values):.4f}")
    
    print(f"\nReward Statistics:")
    print(f"  Mean: {np.mean(rewards_sequence):.1f}")
    print(f"  Std: {np.std(rewards_sequence):.1f}")
    print(f"  Total: {np.sum(rewards_sequence):.1f}")
    
    plt.show()


def hyperparameter_grid_search():
    """Run grid search over hyperparameters"""
    
    tracker = DQNExperimentTracker("hyperparameter_search")
    
    # Define parameter grid
    param_grid = {
        "low_lr": {"lr": 1e-4, "gamma": 0.99, "batch_size": 64},
        "high_lr": {"lr": 1e-2, "gamma": 0.99, "batch_size": 64},
        "low_gamma": {"lr": 1e-3, "gamma": 0.95, "batch_size": 64},
        "high_gamma": {"lr": 1e-3, "gamma": 0.999, "batch_size": 64},
        "small_batch": {"lr": 1e-3, "gamma": 0.99, "batch_size": 32},
        "large_batch": {"lr": 1e-3, "gamma": 0.99, "batch_size": 128},
    }
    
    for config_name, config_dict in param_grid.items():
        tracker.run_training(config_name, config_dict, episodes=100)
    
    tracker.compare_configurations()
    tracker.print_summary()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Advanced DQN utilities')
    parser.add_argument('command', choices=['analyze', 'grid_search'],
                       help='Command to run')
    parser.add_argument('--model', type=str, default='dqn_final_model.pt',
                       help='Model path for analysis')
    parser.add_argument('--episodes', type=int, default=5,
                       help='Number of episodes')
    
    args = parser.parse_args()
    
    if args.command == 'analyze':
        analyze_agent_behavior(args.model, args.episodes)
    elif args.command == 'grid_search':
        hyperparameter_grid_search()
