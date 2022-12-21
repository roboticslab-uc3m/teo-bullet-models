from warnings import simplefilter 
simplefilter(action='ignore', category=FutureWarning)
from tensorflow.python.util import deprecation
deprecation._PRINT_DEPRECATION_WARNINGS = False
import os
import tensorflow as tf
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))

from teoGymEnv import TeoGymEnv
import time

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.cmd_util import make_vec_env
from stable_baselines.common.evaluation import evaluate_policy
from stable_baselines.common.vec_env import VecVideoRecorder, DummyVecEnv
from stable_baselines import PPO2

model_file = "Mlp_PP02_distance_reward_human_limits"
video_folder = 'logs/videos/'
video_length = 5*240

def main():
  environment =  TeoGymEnv(renders=False, isDiscrete=False)
  env = DummyVecEnv([lambda: environment])
  # Load the trained agent

  env = VecVideoRecorder(env, video_folder,
                       record_video_trigger=lambda x: x == 0, video_length=video_length,
                       name_prefix="Teo-{}".format(model_file))

  model = PPO2.load(model_file, env = env)

  obs = env.reset()
  for i in range(video_length+1):
    t0 = time.time()
    action, _states = model.predict(obs)
    t1 = time.time()
    obs, rewards, dones, info = env.step(action)
    t2 = time.time()
    env.render()
    t3 = time.time()
    print("Prediction = {:.3}".format(t1-t0))
    print("Simulation step = {:.3}".format(t2-t1))
    print("Render = {:.3}".format(t3-t2))
  env.close()

if __name__ == "__main__":
  main()