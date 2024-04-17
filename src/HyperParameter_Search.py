import numpy as np
from q_learning.src.QLearnAgent import QLearningAgent
from q_learning.src.CoLearnEnvironment import CoLearn


def hyperSearch():
    agent = QLearningAgent(env=CoLearn())
    agent.load_q_table(directory="Co-Learning-KUKA-RL/Q_tables/q_table_solved_100000_1.npy")
    q_table_solved_mean = agent.q_table.mean()

    mean = 100

    lr = 0.1
    ef = 0.1
    td = 0.1

    learning_rate = 0
    exploration_factor = 0
    trace_decay = 0

    lr_avg = np.array([])
    exp_avg = np.array([])
    trc_avg = np.array([])

    for count in range(100):
        learning_rate = 0
        exploration_factor = 0
        trace_decay = 0
        mean = 100
        print("top loop:",count,"/100")
        for i in range(10):
            a = lr + i/10
            for j in range(10):
                b = ef + j/10
                for k in range(10):
                    c = td + k/10
                    agent2 = QLearningAgent(env=CoLearn())
                    agent2.train(n_steps=1,learning_rate=a,discount_factor=0.2,exploration_factor=b,trace_decay=c,replacement = True)
                    mean_agent2 = agent2.q_table.mean()
                    del agent2

                    if mean_agent2 == np.NaN:
                        pass
                    else:
                        mean_previous = q_table_solved_mean - mean_agent2
                        if mean_previous < mean and mean_previous > 0:
                            mean = mean_previous
                            learning_rate = a
                            exploration_factor = b
                            trace_decay = c
        lr_avg =np.append(lr_avg,learning_rate)
        exp_avg =np.append(exp_avg,exploration_factor)
        trc_avg =np.append(trc_avg,trace_decay)


    print(lr_avg.mean())
    print(exp_avg.mean())
    print(trc_avg.mean())

def epsilon_decay_search(): #TODO: fix functionality, now it always gives lowest value for epsilon and epislon_decay
    agent = QLearningAgent(env=CoLearn())
    agent.load_q_table(directory="Co-Learning-KUKA-RL/Q_tables/q_table_solved_100000_1.npy")
    q_table_solved_mean = agent.q_table.mean()

    mean = 100
    e = 0.95
    decay_factor = 0.99

    epsilon = np.array([])
    epsilon_decay = np.array([])

    for count in range(10):
        mean = 100
        epsilon_save = 0
        epsilon_decay_save = 0
        print("top loop:",count,"/10")
        for j in range(10):
            b = decay_factor - j/100
            for i in range(94):
                a = max(e - i/100,0.1)
                init = a
                agent2 = QLearningAgent(env=CoLearn())
                for _ in range(10):
                    a *= b
                    agent2.train_real_time(learning_rate=0.9,discount_factor=0.2,exploration_factor=a,trace_decay=0.6,replacement = True)
                
            mean_agent2 = agent2.q_table.mean()
            if mean_agent2 == np.NaN:
                pass
            else:
                mean_previous = q_table_solved_mean - mean_agent2
                if mean_previous < mean and mean_previous > 0:
                    epsilon_save=init
                    epsilon_decay_save=b
                    
               
        epsilon = np.append(epsilon,epsilon_save)
        epsilon_decay = np.append(epsilon_decay,epsilon_decay_save)
    print("epsilon_save",epsilon.mean())
    print("epsilon_decay",epsilon_decay.mean())

#epsilon_decay_search()