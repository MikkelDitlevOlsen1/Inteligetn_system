import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from collections import deque
import random
from pathlib import Path
import sys

# Add MPC to path
sys.path.insert(0, str(Path(__file__).parent / "MPC"))

from MPC.map_simulation import MapSimulation
from MPC.map import random_map_generater
NUMBER_OF_CLEANERS = 4
NUMBER_OF_WINDOWS = 30
# Import matplotlib only when needed
matplotlib_available = False
try:
    import matplotlib.pyplot as plt
    matplotlib_available = True
except ImportError:
    print("Warning: matplotlib not available, plotting disabled")

class DQN(nn.Module):
    """Deep Q Network for state-action value estimation"""
    def __init__(self, state_dim, action_dim, hidden_dims=(256, 256)):
        super(DQN, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dims[0]),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(hidden_dims[0], hidden_dims[1]),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(hidden_dims[1], action_dim)
        )
    
    def forward(self, x):
        return self.net(x)


class DQNAgent:
    """DQN Agent for training on MapSimulation environment"""
    def __init__(self, state_dim, action_dim, lr=1e-3, gamma=0.99, 
                 epsilon=1.0, epsilon_min=0.05, epsilon_decay=0.995, 
                 memory_size=10000, batch_size=64, update_frequency=4):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.memory = deque(maxlen=memory_size)
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        self.batch_size = batch_size
        self.update_frequency = update_frequency
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # Networks
        self.model = DQN(state_dim, action_dim).to(self.device)
        self.target_model = DQN(state_dim, action_dim).to(self.device)
        self.target_model.load_state_dict(self.model.state_dict())
        self.target_model.eval()
        
        # Optimizer and loss
        self.optimizer = optim.Adam(self.model.parameters(), lr=lr)
        self.loss_fn = nn.MSELoss()
        
        # Training tracking
        self.update_target_steps = 500
        self.step_count = 0
        self.train_losses = []
        
        print(f"Device: {self.device}")
        print(f"Model parameters: {sum(p.numel() for p in self.model.parameters())}")

    def remember(self, state, action, reward, next_state, done):
        """Store transition in replay memory"""
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state, training=True):
        """Select action using epsilon-greedy policy"""
        if training and np.random.rand() < self.epsilon:
            return np.random.randint(self.action_dim)
        
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.model(state_tensor)
        return q_values.argmax(dim=1).item()

    def replay(self):
        """Experience replay for training"""
        if len(self.memory) < self.batch_size:
            return None
        
        batch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        
        states = torch.FloatTensor(states).to(self.device)
        actions = torch.LongTensor(actions).unsqueeze(1).to(self.device)
        rewards = torch.FloatTensor(rewards).unsqueeze(1).to(self.device)
        next_states = torch.FloatTensor(next_states).to(self.device)
        dones = torch.FloatTensor(dones).unsqueeze(1).to(self.device)

        # Current Q-values
        q_values = self.model(states).gather(1, actions)
        
        # Target Q-values
        with torch.no_grad():
            next_q_values = self.target_model(next_states).max(1, keepdim=True)[0]
            target = rewards + self.gamma * next_q_values * (1 - dones)
        
        # Compute loss and backprop
        loss = self.loss_fn(q_values, target)
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.model.parameters(), 1.0)
        self.optimizer.step()
        
        self.step_count += 1
        
        # Update target network periodically
        if self.step_count % self.update_target_steps == 0:
            self.target_model.load_state_dict(self.model.state_dict())
            print(f"Target network updated at step {self.step_count}")
        
        # Decay epsilon
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
        
        return loss.item()

    def save(self, filepath):
        """Save model checkpoint"""
        torch.save({
            'model_state': self.model.state_dict(),
            'target_model_state': self.target_model.state_dict(),
            'optimizer_state': self.optimizer.state_dict(),
            'step_count': self.step_count,
            'epsilon': self.epsilon,
        }, filepath)
        print(f"Model saved to {filepath}")

    def load(self, filepath):
        """Load model checkpoint"""
        checkpoint = torch.load(filepath, map_location=self.device)
        self.model.load_state_dict(checkpoint['model_state'])
        self.target_model.load_state_dict(checkpoint['target_model_state'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state'])
        self.step_count = checkpoint['step_count']
        self.epsilon = checkpoint['epsilon']
        print(f"Model loaded from {filepath}")


def train(agent, env, episodes=500, max_steps=200, save_interval=50):
    """Train the DQN agent"""
    episode_rewards = []
    episode_lengths = []
    
    print("\n" + "="*60)
    print("Starting DQN Training")
    print("="*60 + "\n")
    
    for ep in range(episodes):
        # Reset environment
        env = MapSimulation(random_map_generater(num_cleaners=NUMBER_OF_CLEANERS, num_windows=NUMBER_OF_WINDOWS), real_time=False)
        state = env.get_dqn_state()
        
        episode_reward = 0
        episode_loss = 0
        steps = 0
        
        for t in range(max_steps):
            # Select and perform action
            action = agent.act(state, training=True)
            next_state, reward, done, _ = env.rl_step(action)
            
            # Store transition and train
            agent.remember(state, action, reward, next_state, done)
            if t % agent.update_frequency == 0:
                loss = agent.replay()
                if loss is not None:
                    episode_loss += loss
            
            state = next_state
            episode_reward += reward
            steps += 1
            
            if done:
                break
        
        episode_rewards.append(episode_reward)
        episode_lengths.append(steps)
        avg_loss = episode_loss / max(steps // agent.update_frequency, 1)
        
        # Print progress
        if (ep + 1) % 10 == 0:
            avg_reward = np.mean(episode_rewards[-10:])
            avg_length = np.mean(episode_lengths[-10:])
            print(f"Episode {ep+1:4d}/{episodes} | Reward: {episode_reward:8.1f} (avg: {avg_reward:8.1f}) | "
                  f"Length: {steps:3d} (avg: {avg_length:6.1f}) | Loss: {avg_loss:8.4f} | "
                  f"Epsilon: {agent.epsilon:.4f}")
        
        # Save checkpoint
        if (ep + 1) % save_interval == 0:
            checkpoint_path = f"dqn_checkpoint_ep{ep+1}.pt"
            agent.save(checkpoint_path)
    
    return episode_rewards, episode_lengths


def test(agent, env, episodes=10, max_steps=200, render=False):
    """Test the trained agent"""
    print("\n" + "="*60)
    print("Testing DQN Agent")
    print("="*60 + "\n")
    
    test_rewards = []
    test_lengths = []
    
    for ep in range(episodes):
        env = MapSimulation(random_map_generater(num_cleaners=NUMBER_OF_CLEANERS, num_windows=NUMBER_OF_WINDOWS), real_time=False)
        state = env.get_dqn_state()
        
        episode_reward = 0
        steps = 0
        
        for t in range(max_steps):
            action = agent.act(state, training=False)
            next_state, reward, done, _ = env.rl_step(action)
            
            state = next_state
            episode_reward += reward
            steps += 1
            
            if done:
                break
        
        test_rewards.append(episode_reward)
        test_lengths.append(steps)
        print(f"Test Episode {ep+1:2d}/{episodes} | Reward: {episode_reward:8.1f} | Length: {steps:3d}")
    
    print(f"\nAverage Test Reward: {np.mean(test_rewards):.1f} ± {np.std(test_rewards):.1f}")
    print(f"Average Test Length: {np.mean(test_lengths):.1f} ± {np.std(test_lengths):.1f}")
    
    return test_rewards, test_lengths


def plot_training_results(episode_rewards, episode_lengths, save_path="training_results.png"):
    """Plot training results"""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Episode rewards
    axes[0].plot(episode_rewards, alpha=0.6, label='Episode Reward')
    # Moving average
    window = 10
    if len(episode_rewards) > window:
        moving_avg = np.convolve(episode_rewards, np.ones(window)/window, mode='valid')
        axes[0].plot(range(window-1, len(episode_rewards)), moving_avg, linewidth=2, label=f'{window}-Episode Average')
    axes[0].set_xlabel('Episode')
    axes[0].set_ylabel('Total Reward')
    axes[0].set_title('Training Rewards')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Episode lengths
    axes[1].plot(episode_lengths, alpha=0.6, label='Episode Length')
    if len(episode_lengths) > window:
        moving_avg = np.convolve(episode_lengths, np.ones(window)/window, mode='valid')
        axes[1].plot(range(window-1, len(episode_lengths)), moving_avg, linewidth=2, label=f'{window}-Episode Average')
    axes[1].set_xlabel('Episode')
    axes[1].set_ylabel('Steps')
    axes[1].set_title('Episode Lengths')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=100, bbox_inches='tight')
    print(f"\nTraining results saved to {save_path}")
    plt.show()


if __name__ == "__main__":
    # Configuration
    random.seed(42)
    np.random.seed(42)
    torch.manual_seed(42)
    
    # Create environment to get state/action dimensions
    print("Initializing environment...")
    random_map = random_map_generater(num_cleaners=NUMBER_OF_CLEANERS, num_windows=NUMBER_OF_WINDOWS)
    max_map=random_map_generater(num_cleaners=4, num_windows=50)
    env = MapSimulation(random_map, real_time=False)
    max_env = MapSimulation(max_map, real_time=False)

    state_dim = len(env.get_dqn_state())
    action_dim = len(max_env.new_build_drone_actions())
    print(f"State dimension: {state_dim}")
    print(f"Action dimension: {action_dim}")
    # Create and train agent
    agent = DQNAgent(state_dim, action_dim, lr=1e-3, gamma=0.99, 
                     epsilon=1.0, epsilon_min=0.05, epsilon_decay=0.995,
                     memory_size=15000, batch_size=64, update_frequency=4)
    
    # Train
    episode_rewards, episode_lengths = train(agent, env, episodes=200, max_steps=150, save_interval=50)
    
    # Save final model
    agent.save("dqn_final_model.pt")
    
    # Test
    test_rewards, test_lengths = test(agent, env, episodes=20, max_steps=200)
    
    # Plot results
    plot_training_results(episode_rewards, episode_lengths)
