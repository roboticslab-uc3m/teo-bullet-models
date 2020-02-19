from warnings import simplefilter 
simplefilter(action='ignore', category=FutureWarning)

from tensorflow.python.util import deprecation
deprecation._PRINT_DEPRECATION_WARNINGS = False
import os
import tensorflow as tf

# stop deprecation warnings for daily use
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)


#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))

from teoGymEnv import TeoGymEnv

import time
import numpy as np 

from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines.common.cmd_util import make_vec_env
from stable_baselines import results_plotter
from stable_baselines.results_plotter import load_results, ts2xy

from stable_baselines import PPO2
from stable_baselines.common.policies import MlpPolicy

best_mean_reward, n_steps = -np.inf, 0

log_dir = "MLP_model_log"

def callback(_locals, _globals):
    """
    Callback called at each step (for DQN an others) or after n steps (see ACER or PPO2)
    :param _locals: (dict)
    :param _globals: (dict)
    """
    global n_steps, best_mean_reward
    # Print stats every 10000 calls
    if (n_steps + 1) % 10000 == 0:
        # Evaluate policy training performance
        x, y = ts2xy(load_results(log_dir), 'timesteps')
        if len(x) > 0:
            mean_reward = np.mean(y[-100:])
            print(x[-1], 'timesteps')
            print("Best mean reward: {:.2f} - Last mean reward per episode: {:.2f}".format(best_mean_reward, mean_reward))

            # New best model, you could save the agent here
            if mean_reward > best_mean_reward and (n_steps + 1) % 5000000 == 0:
                best_mean_reward = mean_reward
                # Example for saving best model
                print("Saving new best model")
                _locals['self'].save(log_dir + str(n_steps) + '_steps_model.pkl')
            if mean_reward > best_mean_reward and (n_steps + 1) % 10000 == 0:
                best_mean_reward = mean_reward
                # Example for saving best model
                print("Saving new best model")
                _locals['self'].save(log_dir + 'best_model.pkl')
    n_steps += 1
    return True

def main():

  environment = TeoGymEnv(renders=False, isDiscrete=False)
  env = SubprocVecEnv([lambda: environment]*12)  # The algorithms require a vectorized environment to run


  model = PPO2(MlpPolicy, env, verbose=1,
               n_steps=512,
               tensorboard_log="log/")
  
  t1 = time.time()
  model.learn(total_timesteps=int(3e6), callback=callback)
  t2 = time.time()
  print("Total training time   = {}".format(t2-t1))


  model.save("Mlp_PP02_distance_reward_human_limits")


if __name__ == "__main__":
  main()
