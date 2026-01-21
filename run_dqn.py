"""
Run trained DQN agent on map simulation with visualization
"""

import torch
import numpy as np
from pathlib import Path
import sys

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
from train_dqn import DQNAgent


def run_agent(model_path="dqn_final_model.pt", num_episodes=5, max_steps=100, visualize=True):
    """Run trained DQN agent and optionally visualize"""
    
    # Initialize environment
    random_map=random_map_generater(num_cleaners=2, num_windows=15)
    max_map=random_map_generater(num_cleaners=4, num_windows=50)
    max_env=MapSimulation(max_map, real_time=False)
    env = MapSimulation(random_map, real_time=False)
    sim = MapSimulation(random_map, real_time=False )
    sim.run(steps=max_steps)
    state_dim = len(env.get_dqn_state())
    action_dim = len(max_env.new_build_drone_actions())
    print(f"State dimension: {state_dim}, Action dimension: {action_dim}")
    # Create and load agent
    agent = DQNAgent(state_dim, action_dim)
    
    if Path(model_path).exists():
        agent.load(model_path)
        print(f"Loaded model from {model_path}")
    else:
        print(f"Warning: Model file {model_path} not found. Using untrained agent.")
    
    print("\n" + "="*60)
    print("Running DQN Agent on Map Simulation")
    print("="*60 + "\n")
    
    episode_rewards = []
    episode_windows_cleaned = []
    
    for ep in range(num_episodes):
        # Create new environment
        env = MapSimulation(random_map_generater(num_cleaners=2, num_windows=15), real_time=False)
        state = env.get_dqn_state()
        
        episode_reward = 0
        steps = 0
        windows_cleaned = 0
        trajectory = []
        
        for t in range(max_steps):
            # Get action from agent
            action = agent.act(state, training=False)
            next_state, reward, done, _ = env.rl_step(action)
            
            # Track trajectory
            trajectory.append({
                'state': state.copy(),
                'action': action,
                'reward': reward,
                'done': done
            })
            
            state = next_state
            episode_reward += reward
            steps += 1
            
            # Count windows cleaned (simplified: check if any reward was from cleaning)
            if reward > 100:  # Cleaning reward
                windows_cleaned += reward // 1000
            
            if done:
                break
        
        episode_rewards.append(episode_reward)
        episode_windows_cleaned.append(windows_cleaned)
        
        print(f"Episode {ep+1:2d}/{num_episodes} | Reward: {episode_reward:8.1f} | "
              f"Steps: {steps:3d} | Windows Cleaned: {windows_cleaned}")
    
    # Summary statistics
    print("\n" + "="*60)
    print("Summary Statistics")
    print("="*60)
    print(f"Average Reward: {np.mean(episode_rewards):.1f} ± {np.std(episode_rewards):.1f}")
    print(f"Average Episode Length: {np.mean([ep for ep in range(len(episode_rewards))]):.1f}")
    print(f"Average Windows Cleaned: {np.mean(episode_windows_cleaned):.1f}")
    
    if visualize:
        if not matplotlib_available:
            print("Warning: matplotlib not available, skipping visualization")
            return episode_rewards, episode_windows_cleaned
            
        # Plot results
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        
        # Episode rewards
        axes[0].bar(range(1, num_episodes + 1), episode_rewards, color='steelblue', alpha=0.7)
        axes[0].axhline(y=np.mean(episode_rewards), color='red', linestyle='--', label='Mean')
        axes[0].set_xlabel('Episode')
        axes[0].set_ylabel('Total Reward')
        axes[0].set_title('DQN Agent Performance')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        
        # Windows cleaned
        axes[1].bar(range(1, num_episodes + 1), episode_windows_cleaned, color='green', alpha=0.7)
        axes[1].axhline(y=np.mean(episode_windows_cleaned), color='red', linestyle='--', label='Mean')
        axes[1].set_xlabel('Episode')
        axes[1].set_ylabel('Windows Cleaned')
        axes[1].set_title('Window Cleaning Performance')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig("dqn_test_results.png", dpi=100, bbox_inches='tight')
        print("\nResults saved to dqn_test_results.png")
        plt.show()
    
    return episode_rewards, episode_windows_cleaned


def compare_random_vs_trained(model_path="dqn_final_model.pt", num_episodes=10, max_steps=200):
    """Compare random agent vs trained DQN agent"""
    
    # Initialize environment
    env = MapSimulation(random_map_generater(num_cleaners=2, num_windows=5), real_time=False)
    state_dim = len(env.get_dqn_state())
    action_dim = len(env.new_build_drone_actions())
    
    # Load trained agent
    agent = DQNAgent(state_dim, action_dim)
    if Path(model_path).exists():
        agent.load(model_path)
    
    print("\n" + "="*60)
    print("Comparing Random Agent vs Trained DQN Agent")
    print("="*60 + "\n")
    
    random_rewards = []
    trained_rewards = []
    
    for ep in range(num_episodes):
        # Test random agent
        env_random = MapSimulation(random_map_generater(num_cleaners=2, num_windows=5), real_time=False)
        state = env_random.get_dqn_state()
        random_reward = 0
        
        for t in range(max_steps):
            action = np.random.randint(action_dim)
            next_state, reward, done, _ = env_random.rl_step(action)
            state = next_state
            random_reward += reward
            if done:
                break
        
        random_rewards.append(random_reward)
        
        # Test trained agent
        env_trained = MapSimulation(random_map_generater(num_cleaners=2, num_windows=5), real_time=False)
        state = env_trained.get_dqn_state()
        trained_reward = 0
        
        for t in range(max_steps):
            action = agent.act(state, training=False)
            next_state, reward, done, _ = env_trained.rl_step(action)
            state = next_state
            trained_reward += reward
            if done:
                break
        
        trained_rewards.append(trained_reward)
        
        print(f"Episode {ep+1:2d} | Random: {random_reward:8.1f} | Trained: {trained_reward:8.1f} | "
              f"Improvement: {trained_reward - random_reward:+8.1f}")
    
    print("\n" + "="*60)
    print("Statistics")
    print("="*60)
    print(f"Random Agent - Mean: {np.mean(random_rewards):.1f}, Std: {np.std(random_rewards):.1f}")
    print(f"Trained DQN  - Mean: {np.mean(trained_rewards):.1f}, Std: {np.std(trained_rewards):.1f}")
    print(f"Improvement: {np.mean(trained_rewards) - np.mean(random_rewards):.1f} "
          f"({(np.mean(trained_rewards) / np.mean(random_rewards) - 1) * 100:.1f}%)")
    
    # Plot comparison
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Bar comparison
    x = np.arange(num_episodes)
    width = 0.35
    axes[0].bar(x - width/2, random_rewards, width, label='Random', alpha=0.7)
    axes[0].bar(x + width/2, trained_rewards, width, label='Trained DQN', alpha=0.7)
    axes[0].set_xlabel('Episode')
    axes[0].set_ylabel('Total Reward')
    axes[0].set_title('Random vs Trained DQN Agent')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3, axis='y')
    
    # Box plot
    axes[1].boxplot([random_rewards, trained_rewards], labels=['Random', 'Trained DQN'])
    axes[1].set_ylabel('Total Reward')
    axes[1].set_title('Reward Distribution')
    axes[1].grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig("comparison_random_vs_trained.png", dpi=100, bbox_inches='tight')
    print("\nComparison plot saved to comparison_random_vs_trained.png")
    plt.show()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Run trained DQN agent on map simulation')
    parser.add_argument('--model', type=str, default='dqn_final_model.pt', help='Path to trained model')
    parser.add_argument('--episodes', type=int, default=1, help='Number of episodes to run')
    parser.add_argument('--steps', type=int, default=100, help='Max steps per episode')
    parser.add_argument('--compare', action='store_true', help='Compare with random agent')
    parser.add_argument('--no-viz', action='store_true', help='Disable visualization')
    
    args = parser.parse_args()
    
    if args.compare:
        compare_random_vs_trained(args.model, args.episodes, args.steps)
    else:
        run_agent(args.model, args.episodes, args.steps, visualize=not args.no_viz)
