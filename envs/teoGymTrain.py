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
from stable_baselines.common.vec_env import SubprocVecEnv
from stable_baselines.common.cmd_util import make_vec_env
from stable_baselines import PPO2


def main():

  environment = TeoGymEnv(renders=False, isDiscrete=False)
  env = SubprocVecEnv([lambda: environment])  # The algorithms require a vectorized environment to run

  model = PPO2(MlpPolicy, env, verbose=1,
               n_steps=512,
               tensorboard_log="log/")
  model.learn(total_timesteps=int(50e2))

  model.save("Mlp_PP02_walking")


if __name__ == "__main__":
  main()
