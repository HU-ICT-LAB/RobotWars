"""Main function for the pipeline."""

from classes import DatabaseSAC, baseline_reward_function
from stable_baselines3.common.callbacks import CheckpointCallback
from gym.spaces import Discrete, Box


OBS_VALUES = 12
ACT_VALUES = 6


def main():
    """Run the pipeline and output the model locally."""
    version = 0
    algorithm = DatabaseSAC("MlpPolicy", baseline_reward_function, Discrete(OBS_VALUES), Box(0, 1, shape=(ACT_VALUES,)))
    while True:
        algorithm.learn(10001, callback=CheckpointCallback(10000, "policy/", "rl-model-v{}".format(version)))
        # TODO: Moet uiteindelijk vervangen worden door een callback die direct het algoritme opslaat in de database.
        version += 1


if __name__ == "__main__":
    main()
