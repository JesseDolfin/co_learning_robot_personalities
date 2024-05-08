import numpy as np

class RoboticArmOption:
    def __init__(self, phase_policy, initiation_set, termination_set, gamma):
        self.phase_policy = phase_policy
        self.initiation_set = initiation_set
        self.termination_set = termination_set
        self.gamma = gamma
        self.action_name = phase_policy(None)  # Store the action name directly

    def update_option(self, phase_policy=None, initiation_set=None, termination_set=None, gamma=None):
        if phase_policy:
            self.phase_policy = phase_policy
        if initiation_set:
            self.initiation_set = initiation_set
        if termination_set:
            self.termination_set = termination_set
        if gamma:
            self.gamma = gamma

class Environment:
    def __init__(self):
        self.states = ['home', 'T1', 'T2', 'A', 'B', 'C', 'D']
        self.actions = ['move_at_T1_T2', 'move_to_A_B', 'rotate_to_C_D']
        self.rewards = {'C': 1, 'D': 1}

    def step(self, state, action):
        if action not in self.actions:
            raise ValueError("Invalid action")
        
        if action == 'move_at_T1_T2':
            next_state = np.random.choice(['T1', 'T2'])
            reward = 0
        elif action == 'move_to_A_B':
            next_state = np.random.choice(['A', 'B'])
            reward = 0
        else:  # 'orient_to_C_D'
            next_state = np.random.choice(['C', 'D'])
            reward = self.rewards.get(next_state, 0)
        
        return next_state, reward

class ReinforcementLearningOptions:
    def __init__(self, environment, options, num_episodes=1000, alpha=0.1, gamma=0.9, epsilon=0.1, alpha_decay=None):
        self.environment = environment
        self.options = options
        self.num_episodes = num_episodes
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.alpha_decay = alpha_decay
        self.Q = {option.action_name: {state: {action: 0.0 for action in environment.actions} for state in environment.states} for option in options}

    def epsilon_greedy_policy(self, state, option_index):
        if np.random.rand() < self.epsilon:
            return np.random.choice(self.environment.actions)
        else:
            option_key = list(self.Q.keys())[option_index]
            return max(self.Q[option_key][state], key=self.Q[option_key][state].get)

    def q_learning_with_options(self):
        for episode in range(self.num_episodes):
            state = 'home'
            option_index = 0

            while True:
                option = self.options[option_index]
                option_key = option.action_name
                while state not in option.termination_set:
                    action = self.epsilon_greedy_policy(state, option_index)
                    next_state, reward = self.environment.step(state, action)
                    self.Q[option_key][state][action] += self.alpha * (
                            reward + self.gamma * max(self.Q[option_key][next_state].values()) - self.Q[option_key][state][action])
                    state = next_state

                option_index += 1
                if option_index >= len(self.options):
                    break

                state = option.termination_set[0]

                if self.alpha_decay:
                    self.alpha *= self.alpha_decay

    def print_q_table(self):
        for option_key, option_q_table in self.Q.items():
            print(f"Option: {option_key}")
            for state, actions in option_q_table.items():
                print(f"  State: {state}")
                for action, q_value in actions.items():
                    print(f"    Action: {action}, Q-value: {q_value}")



# Usage example:

# Create an instance of the Environment
env = Environment()

# Define options
move_at_T1_T2_option = RoboticArmOption(lambda state: 'move_at_T1_T2', ['home'], ['T1', 'T2'], 0.9)
move_to_A_B_option = RoboticArmOption(lambda state: 'move_to_A_B', ['T1','T2'], ['A', 'B'], 0.9)
rotate_to_A_B_option = RoboticArmOption(lambda state: 'rotate_to_C_D', ['A','B'], ['C', 'D'], 0.9)
options = [move_at_T1_T2_option,move_to_A_B_option,rotate_to_A_B_option]

# Create an instance of ReinforcementLearningOptions with options and alpha decay
rl_agent = ReinforcementLearningOptions(env, options, alpha=0.5, alpha_decay=0.95)

# Run Q-learning with options
rl_agent.q_learning_with_options()


rl_agent.print_q_table()


