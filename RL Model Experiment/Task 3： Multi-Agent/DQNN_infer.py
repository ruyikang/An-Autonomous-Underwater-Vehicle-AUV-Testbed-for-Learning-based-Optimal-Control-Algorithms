import numpy as np
from stable_baselines3 import DQN
import joblib
import sklearn

# rospy.init_node('DdpgDeepleng_docker', anonymous=True)
# arduinoData  = serial.Serial('com3',115200)
if __name__ == "__main__":
    # env_id = "DeeplengDocking-v2"
    num = "N1"
    step = "11000000"  # N1： 1100， N2： 1600
    # env = make_vec_env(env_id, vec_env_cls=SubprocVecEnv)
    # env = VecNormalize.load(f"DQN/checkpoint_{num}/rl_model_vecnormalize_{step}_steps.pkl", env)

    model = joblib.load(r"DTC_improve2.pkl")
    # model = DQN.load("DQN/checkpoints/rl_model_50000_steps.zip")

    # obs = env.reset()


    for i in range(2000):
        obs = [[51, 11, -167, 1]]
        print("{}".format(np.arctan2(-11, -51) * 180 / np.pi))
        action = model.predict(obs)
        action = str(action)
        print("action: {}".format(action))


