#!/usr/bin/env python3
"""
Complete DQN Demo Pipeline
Trains and runs DQN agent with one command
"""

import argparse
import sys
from pathlib import Path

# Add MPC to path
sys.path.insert(0, str(Path(__file__).parent / "MPC"))
7

from train_dqn import DQNAgent, train, test
from run_dqn import run_agent, compare_random_vs_trained
from MPC.map_simulation import MapSimulation
from MPC.map import random_map_generater
import torch
import numpy as np


def print_banner(text):
    """Print formatted banner"""
    width = 70
    print("\n" + "="*width)
    print(text.center(width))
    print("="*width + "\n")


def check_dependencies():
    """Check if required packages are installed"""
    print_banner("Checking Dependencies")
    
    required = {
        'torch': 'PyTorch',
        'numpy': 'NumPy',
        'matplotlib': 'Matplotlib'
    }
    
    missing = []
    for module, name in required.items():
        try:
            __import__(module)
            print(f"✓ {name} is installed")
        except ImportError:
            print(f"✗ {name} is NOT installed")
            missing.append(module)
    
    if missing:
        print(f"\nPlease install missing packages:")
        print(f"  pip install {' '.join(missing)}")
        return False
    
    print(f"\nGPU Available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"GPU Device: {torch.cuda.get_device_name(0)}")
    
    return True


def quick_train(episodes=50):
    """Quick training for testing"""
    print_banner(f"Quick Training ({episodes} episodes)")
    
    # Initialize
    env = MapSimulation(random_map_generater(num_cleaners=2, num_windows=5), real_time=False)
    state_dim = len(env.get_dqn_state())
    action_dim = len(env.new_build_drone_actions())
    
    print(f"State dimension: {state_dim}")
    print(f"Action dimension: {action_dim}\n")
    
    # Create agent
    agent = DQNAgent(state_dim, action_dim, lr=1e-3, gamma=0.99)
    
    # Train
    rewards, lengths = train(agent, env, episodes=episodes, max_steps=150, save_interval=10)
    
    # Save
    agent.save("dqn_quick_model.pt")
    
    print(f"\nTraining complete!")
    print(f"Final average reward (last 10 episodes): {np.mean(rewards[-10:]):.1f}")
    
    return agent, rewards


def full_train(episodes=200):
    """Full training"""
    print_banner(f"Full Training ({episodes} episodes)")
    
    # Initialize
    env = MapSimulation(random_map_generater(num_cleaners=2, num_windows=5), real_time=False)
    state_dim = len(env.get_dqn_state())
    action_dim = len(env.new_build_drone_actions())
    
    print(f"State dimension: {state_dim}")
    print(f"Action dimension: {action_dim}\n")
    
    # Create agent
    agent = DQNAgent(state_dim, action_dim, lr=1e-3, gamma=0.99, 
                     epsilon=1.0, epsilon_min=0.05, epsilon_decay=0.995,
                     memory_size=15000, batch_size=64)
    
    # Train
    rewards, lengths = train(agent, env, episodes=episodes, max_steps=150, save_interval=50)
    
    # Save
    agent.save("dqn_final_model.pt")
    
    print(f"\nTraining complete!")
    print(f"Initial average reward (first 10): {np.mean(rewards[:10]):.1f}")
    print(f"Final average reward (last 10): {np.mean(rewards[-10:]):.1f}")
    print(f"Improvement: {np.mean(rewards[-10:]) - np.mean(rewards[:10]):.1f}")
    
    return agent, rewards


def test_agent(model_path="dqn_final_model.pt", episodes=5):
    """Test trained agent"""
    print_banner(f"Testing Agent ({episodes} episodes)")
    
    if not Path(model_path).exists():
        print(f"Error: Model file '{model_path}' not found!")
        print("Please train a model first.")
        return
    
    test_rewards, test_lengths = test(Path(model_path), episodes=episodes, max_steps=200)
    
    print(f"\nTest Statistics:")
    print(f"  Average Reward: {np.mean(test_rewards):.1f} ± {np.std(test_rewards):.1f}")
    print(f"  Average Episode Length: {np.mean(test_lengths):.1f}")


def compare_agents(model_path="dqn_final_model.pt", episodes=10):
    """Compare trained vs random agent"""
    print_banner(f"Comparing Trained vs Random Agent ({episodes} episodes each)")
    
    if not Path(model_path).exists():
        print(f"Error: Model file '{model_path}' not found!")
        return
    
    compare_random_vs_trained(model_path, episodes=episodes)


def interactive_demo():
    """Interactive demo menu"""
    while True:
        print("\n" + "="*60)
        print("DQN Agent Training & Testing Demo".center(60))
        print("="*60)
        print("\nOptions:")
        print("  1. Check Dependencies")
        print("  2. Quick Training (50 episodes) - ~5 min")
        print("  3. Full Training (200 episodes) - ~20 min")
        print("  4. Test Trained Agent")
        print("  5. Compare Trained vs Random")
        print("  6. Run with Visualization")
        print("  7. Exit")
        print()
        
        choice = input("Enter your choice (1-7): ").strip()
        
        if choice == "1":
            check_dependencies()
        elif choice == "2":
            quick_train(50)
        elif choice == "3":
            full_train(200)
        elif choice == "4":
            test_agent()
        elif choice == "5":
            compare_agents()
        elif choice == "6":
            run_agent("dqn_final_model.pt", episodes=5, visualize=True)
        elif choice == "7":
            print("\nExiting...")
            break
        else:
            print("Invalid choice. Please try again.")
        
        input("\nPress Enter to continue...")


def main():
    parser = argparse.ArgumentParser(
        description='DQN Agent Training and Testing Pipeline',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python dqn_demo.py --check-deps          # Check dependencies
  python dqn_demo.py --quick-train         # Train for 50 episodes
  python dqn_demo.py --train 200           # Train for 200 episodes
  python dqn_demo.py --test                # Test trained agent
  python dqn_demo.py --compare             # Compare agents
  python dqn_demo.py --run --viz           # Run and visualize
  python dqn_demo.py --interactive         # Interactive menu
        """
    )
    
    parser.add_argument('--check-deps', action='store_true', 
                       help='Check dependencies')
    parser.add_argument('--quick-train', action='store_true',
                       help='Quick training (50 episodes)')
    parser.add_argument('--train', type=int, metavar='EPISODES',
                       help='Full training with specified episodes')
    parser.add_argument('--test', action='store_true',
                       help='Test trained agent')
    parser.add_argument('--compare', action='store_true',
                       help='Compare trained vs random agent')
    parser.add_argument('--run', action='store_true',
                       help='Run trained agent')
    parser.add_argument('--model', type=str, default='dqn_final_model.pt',
                       help='Path to trained model (default: dqn_final_model.pt)')
    parser.add_argument('--episodes', type=int, default=5,
                       help='Number of episodes (default: 5)')
    parser.add_argument('--viz', action='store_true',
                       help='Enable visualization')
    parser.add_argument('--interactive', '-i', action='store_true',
                       help='Interactive menu')
    
    args = parser.parse_args()
    
    # Check if any argument provided
    if len(sys.argv) == 1:
        args.interactive = True
    
    # Execute commands
    if args.check_deps:
        check_dependencies()
    
    if args.quick_train:
        quick_train(50)
    
    if args.train:
        full_train(args.train)
    
    if args.test:
        test_agent(args.model, args.episodes)
    
    if args.compare:
        compare_agents(args.model, args.episodes)
    
    if args.run:
        run_agent(args.model, args.episodes, visualize=args.viz)
    
    if args.interactive:
        interactive_demo()


if __name__ == "__main__":
    main()
