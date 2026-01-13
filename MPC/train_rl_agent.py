import numpy as np
from map_simulation import MapSimulation
from map import random_map_generater
from rl_agent import QLearningAgent

if __name__ == "__main__":
    np.random.seed(42)
    sim = MapSimulation(random_map_generater(num_cleaners=2, num_windows=5), real_time=False)
    # Get number of possible drone actions
    n_actions = len(sim._build_drone_actions(sim.map))
    agent = QLearningAgent(n_actions=n_actions)

    episodes = 10
    max_steps = 100
    for ep in range(episodes):
        sim.map = random_map_generater(num_cleaners=2, num_windows=5)
        state = sim.get_state()
        total_reward = 0
        for step in range(max_steps):
            action = agent.select_action(state)
            next_state, reward, done, new_map = sim.rl_step(action, sim.map)
            agent.update(state, action, reward, next_state, done)
            state = next_state
            sim.map = new_map
            total_reward += reward
            if done:
                break
        print(f"Episode {ep+1}: total reward = {total_reward}, steps = {step+1}")
