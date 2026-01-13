import numpy as np
import random
from map_simulation import MapSimulation

class QLearningAgent:
    def __init__(self, n_actions, alpha=0.1, gamma=0.99, epsilon=0.1):
        self.q_table = {}
        self.n_actions = n_actions
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon

    def get_qs(self, state):
        if state not in self.q_table:
            self.q_table[state] = np.zeros(self.n_actions)
        return self.q_table[state]

    def select_action(self, state):
        if random.random() < self.epsilon:
            return random.randint(0, self.n_actions - 1)
        qs = self.get_qs(state)
        return int(np.argmax(qs))

    def update(self, state, action, reward, next_state, done):
        qs = self.get_qs(state)
        next_qs = self.get_qs(next_state)
        target = reward + (0 if done else self.gamma * np.max(next_qs))
        qs[action] += self.alpha * (target - qs[action])

# Example usage (to be integrated with MapSimulation):
# sim = MapSimulation(...)
# agent = QLearningAgent(n_actions=10)
# state = sim.get_state()
# action = agent.select_action(state)
# next_state, reward, done, _ = sim.rl_step(action)
# agent.update(state, action, reward, next_state, done)
