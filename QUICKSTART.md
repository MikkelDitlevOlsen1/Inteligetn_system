# Quick Start Guide - DQN Agent

## 5-Minute Setup

### 1. Install Dependencies
```bash
pip install torch numpy matplotlib
```

### 2. Train the Model
```bash
python train_dqn.py
```
- Saves trained model as `dqn_final_model.pt`
- Takes ~20 minutes on CPU (3 minutes on GPU)

### 3. Test the Model
```bash
python run_dqn.py --episodes 10
```

### 4. Compare with Random Agent
```bash
python run_dqn.py --compare --episodes 10
```

---

## Command Reference

### Training
```bash
# Standard training (200 episodes)
python train_dqn.py

# Quick training for testing (50 episodes)
python dqn_demo.py --quick-train

# Custom episodes
python dqn_demo.py --train 500
```

### Testing
```bash
# Test trained model (5 episodes)
python run_dqn.py

# Test with 20 episodes
python run_dqn.py --episodes 20

# Test with specific model
python run_dqn.py --model my_model.pt --episodes 10
```

### Comparison
```bash
# Compare random vs trained (10 episodes each)
python run_dqn.py --compare --episodes 10
```

### Analysis
```bash
# Analyze agent behavior
python dqn_utils.py analyze --model dqn_final_model.pt --episodes 5

# Hyperparameter grid search (runs 6 configurations)
python dqn_utils.py grid_search
```

### Interactive Demo
```bash
# Run interactive menu
python dqn_demo.py --interactive

# Or just run without arguments
python dqn_demo.py
```

---

## File Structure

```
Inteligetn_system/
├── train_dqn.py              # Main training script
├── run_dqn.py                # Testing and comparison
├── dqn_utils.py              # Advanced analysis
├── dqn_demo.py               # Interactive demo
├── dqn_agent.py              # Original DQN (reference)
├── requirements.txt          # Dependencies
├── DQN_README.md             # Detailed documentation
├── SETUP_GUIDE.md            # Comprehensive setup
├── QUICKSTART.md             # This file
├── dqn_final_model.pt        # Trained model (after training)
├── dqn_checkpoint_*.pt       # Training checkpoints
├── training_results.png      # Learning curves
└── MPC/                      # Your simulation environment
    ├── map_simulation.py
    ├── map.py
    └── ...
```

---

## Expected Output

### Training
```
Training DQN Agent on Map Simulation
==================================================

Episode  10/200 | Reward:   1234.5 (avg:    1500.2) | Steps: 142 | Loss: 1.2345 | Epsilon: 0.9950
Episode  20/200 | Reward:   2456.8 (avg:    2100.1) | Steps: 135 | Loss: 0.8765 | Epsilon: 0.9900
Episode  30/200 | Reward:   3789.2 (avg:    2650.3) | Steps: 128 | Loss: 0.5234 | Epsilon: 0.9850
...
Training Results:
  - Average final reward: ~5000
  - Improvement over random: ~5-10x
```

### Testing
```
Test Episode 1/5 | Reward:    5234.1 | Length: 95
Test Episode 2/5 | Reward:    4876.2 | Length: 102
Test Episode 3/5 | Reward:    5612.3 | Length: 88
Test Episode 4/5 | Reward:    4945.1 | Length: 105
Test Episode 5/5 | Reward:    5123.4 | Length: 91

Average Test Reward: 5158.2 ± 290.5
Average Test Length: 96.2 ± 7.4
```

### Comparison
```
Episode  1 | Random:  1234.5 | Trained:  5234.1 | Improvement:  +4000.0
Episode  2 | Random:  1456.2 | Trained:  4876.2 | Improvement:  +3420.0
...
Statistics:
Random Agent - Mean: 1456.7, Std: 245.3
Trained DQN  - Mean: 5158.2, Std: 290.5
Improvement: 3701.5 (254.3%)
```

---

## Troubleshooting

### "Module not found: map_simulation"
**Fix:** Run from the parent directory of the MPC folder:
```bash
cd c:\Users\mikke\Documents\Inteligetn_system
python train_dqn.py
```

### Training is very slow
**Solution:** Use GPU (if available):
```python
# Scripts auto-detect GPU, ensure PyTorch has CUDA support
pip install torch torchvision torchaudio pytorch-cuda=12.1 -f https://download.pytorch.org/whl/torch_stable.html
```

### Model file not found when loading
**Fix:** Train first:
```bash
python train_dqn.py  # Creates dqn_final_model.pt
python run_dqn.py    # Now loads the trained model
```

### Out of memory error
**Solution:** Reduce batch size and memory:
```python
# In train_dqn.py, modify:
agent = DQNAgent(state_dim, action_dim, 
                memory_size=5000,   # Reduced from 15000
                batch_size=32)       # Reduced from 64
```

---

## Performance Tips

### To train faster:
- Use GPU (automatically detected)
- Reduce training episodes: `python dqn_demo.py --train 100`
- Increase learning rate in train_dqn.py: `lr=1e-2`

### To get better results:
- Increase training episodes: `python dqn_demo.py --train 500`
- Lower learning rate: `lr=5e-4`
- Train longer epsilon decay: `epsilon_decay=0.998`

### To analyze results:
- Plot training curves: `training_results.png` (auto-generated)
- Analyze behavior: `python dqn_utils.py analyze`
- Compare agents: `python run_dqn.py --compare`

---

## What's Happening

### The DQN Agent Learns To:
1. **Pick up cleaners** at the base station
2. **Fly to windows** that need cleaning
3. **Drop off cleaners** at the right locations
4. **Return to base** for charging when needed
5. **Manage battery** efficiently
6. **Maximize windows** cleaned in available time

### Reward Signal:
- **+1000** for each window cleaned
- **-1000** if cleaner battery depletes
- Small penalties for time/inefficiency

### During Training:
- First episodes: Random, poor performance
- Middle episodes: Learning patterns
- Later episodes: Optimized strategy

---

## Example Usage Scenarios

### Scenario 1: Quick Test (5 minutes)
```bash
python dqn_demo.py --quick-train  # 50 episodes
python run_dqn.py --episodes 5     # Test 5 times
```

### Scenario 2: Standard Training (30 minutes)
```bash
python train_dqn.py               # 200 episodes
python run_dqn.py --episodes 10   # Test 10 times
python run_dqn.py --compare       # Compare with random
```

### Scenario 3: Research/Analysis (2 hours)
```bash
python dqn_utils.py grid_search           # Test 6 configs
python dqn_utils.py analyze --episodes 20 # Detailed analysis
```

### Scenario 4: Interactive Exploration
```bash
python dqn_demo.py --interactive  # Menu-driven interface
```

---

## Next Steps

1. **Understand the results**: Check `training_results.png`
2. **Test the agent**: Run `python run_dqn.py --compare`
3. **Analyze behavior**: Run `python dqn_utils.py analyze`
4. **Fine-tune hyperparameters**: Edit `train_dqn.py` and retrain
5. **Integrate with visualization**: Use `run_with_visualization.py` in MPC/

---

## Documentation

- **DQN_README.md**: Detailed technical documentation
- **SETUP_GUIDE.md**: Comprehensive setup and configuration
- **QUICKSTART.md**: This file (quick reference)

---

**Ready to train?** Start with:
```bash
python train_dqn.py
```

---
Created: January 13, 2026
Status: Ready for use
