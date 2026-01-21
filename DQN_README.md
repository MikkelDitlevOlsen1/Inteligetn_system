# DQN Agent Training and Execution Guide

This project implements a Deep Q Network (DQN) agent to learn and optimize window cleaning tasks in a simulated environment with drones and robot cleaners.

## Files

- **`train_dqn.py`**: Main training script for the DQN agent
- **`run_dqn.py`**: Script to run trained model and compare with random agent
- **`dqn_agent.py`**: Original DQN implementation (reference)

## Quick Start

### 1. Training the DQN Agent

To train a new DQN model:

```bash
python train_dqn.py
```

**What happens:**
- Creates a DQN agent with neural network architecture (256-256 hidden layers)
- Trains for 200 episodes (configurable)
- Uses experience replay and target network for stable learning
- Saves checkpoints every 50 episodes
- Saves final model as `dqn_final_model.pt`
- Generates `training_results.png` showing learning curves

**Training Configuration (in `train_dqn.py`):**
```python
episodes = 200              # Number of training episodes
max_steps = 150             # Max steps per episode
batch_size = 64             # Batch size for experience replay
gamma = 0.99                # Discount factor
epsilon_decay = 0.995       # Epsilon decay rate
learning_rate = 1e-3        # Adam optimizer learning rate
```

### 2. Running the Trained Agent

To test the trained model:

```bash
python run_dqn.py --episodes 5 --steps 200
```

**Options:**
- `--model`: Path to trained model (default: `dqn_final_model.pt`)
- `--episodes`: Number of test episodes (default: 5)
- `--steps`: Max steps per episode (default: 200)
- `--compare`: Compare trained agent vs random agent
- `--no-viz`: Skip visualization

### 3. Compare with Random Agent

To see how much the trained agent improves over random actions:

```bash
python run_dqn.py --compare --episodes 10
```

This will:
- Run both random and trained agents for comparison
- Display performance metrics
- Generate `comparison_random_vs_trained.png`

## Model Architecture

### DQN Network
```
Input (State dimension)
    ↓
Dense(256) + ReLU + Dropout(0.1)
    ↓
Dense(256) + ReLU + Dropout(0.1)
    ↓
Dense(action_dim)
    ↓
Output (Q-values for each action)
```

### Training Algorithm
1. **Action Selection**: ε-greedy policy
   - With probability ε: random action
   - Otherwise: action with highest Q-value
   
2. **Experience Replay**:
   - Store transitions in replay buffer
   - Sample random batches for training
   - Breaks correlation between consecutive samples

3. **Target Network**:
   - Separate network for computing target Q-values
   - Updated periodically from main network
   - Stabilizes training

4. **Loss Function**: Mean Squared Error between predicted and target Q-values

## Environment Details

**State Space:**
- Drone position (x, y, z)
- Drone battery level
- Drone load status
- Base station position
- For each cleaner: position, battery, cleaning status, location
- For each window: cleaning status

**Action Space:**
- Fly to base station
- Charge drone
- Drop off cleaner at base
- Fly to window
- Drop off cleaner at window
- Pick up cleaner

**Reward Function:**
- +1000 for each window cleaned
- -1000 if any cleaner battery depletes
- Small penalty based on time

## Performance Tips

### To improve training:
1. **Increase episodes**: `episodes = 500` for better convergence
2. **Tune learning rate**: Try `lr=5e-4` or `lr=2e-3`
3. **Adjust epsilon decay**: `epsilon_decay=0.998` for slower exploration decay
4. **Increase memory size**: `memory_size=20000` for better replay buffer

### To train faster:
1. Reduce `max_steps` during training
2. Reduce `batch_size` (e.g., 32)
3. Use GPU if available (automatic detection)

## Interpreting Results

### Training Curves
- **Reward plot**: Should generally increase over episodes
- **Episode length plot**: Shows convergence to efficient solutions

### Test Results
- Average Reward: Target reward accumulated per episode
- Windows Cleaned: Number of windows successfully cleaned
- Comparison: Trained agent should significantly outperform random agent

## Troubleshooting

**"Module not found: map_simulation"**
- Ensure you're running from the parent directory of MPC folder
- Scripts automatically add MPC to path

**Memory issues**
- Reduce `memory_size` (e.g., 5000)
- Reduce `batch_size` (e.g., 32)
- Run on GPU by ensuring CUDA is available

**No improvement in training**
- Check state representation in `map_simulation.py`
- Verify reward computation
- Try different learning rates
- Increase training episodes

## Advanced Usage

### Custom Training Script
```python
from train_dqn import DQNAgent, train
from map import random_map_generater

agent = DQNAgent(state_dim=42, action_dim=8, lr=5e-4)
rewards, lengths = train(agent, env, episodes=500, max_steps=200)
agent.save("my_model.pt")
```

### Loading and Fine-tuning
```python
from train_dqn import DQNAgent

agent = DQNAgent(state_dim=42, action_dim=8)
agent.load("dqn_final_model.pt")

# Adjust epsilon for more exploration
agent.epsilon = 0.3

# Continue training
for episode in range(100):
    # ... training loop
```

## References

- Deep Q-Learning: https://www.nature.com/articles/nature14236
- Double DQN improvements available in future versions
- Dueling DQN architecture for potentially better performance

## Notes

- The agent learns through trial and error
- Training time depends on environment complexity and hardware
- GPU acceleration (CUDA) will significantly speed up training
- Model saves checkpoint regularly, so you can stop training anytime
