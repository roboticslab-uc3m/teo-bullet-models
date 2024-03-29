from tensorflow.python.util import deprecation
deprecation._PRINT_DEPRECATION_WARNINGS = False
import os
import tensorflow as tf
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))

from teoGymEnv import TeoGymEnv
import time

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.common.cmd_util import make_vec_env
from stable_baselines.common.evaluation import evaluate_policy
from stable_baselines import PPO2


def main():
  environment =  TeoGymEnv(renders=True, isDiscrete=False)
  env = DummyVecEnv([lambda: environment])
  # Load the trained agent
  model = PPO2.load("Mlp_PP02_walking", env = env)

  # Enjoy trained agent
  obs = env.reset()
  for i in range(100):
    t1 = time.time()
    action, _states = model.predict(obs)
    t2 = time.time()
    obs, rewards, dones, info = env.step(action)
    t3 = time.time()
    env.render()
    t4=time.time()




if __name__ == "__main__":
  main()
