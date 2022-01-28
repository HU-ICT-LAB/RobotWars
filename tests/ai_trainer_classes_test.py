"""Contains tests to verify the main functionality of the custom classes found in ai_trainer/classes.py."""

import unittest
import numpy as np
import gym

from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.buffers import ReplayBuffer
from stable_baselines3.sac import SAC
from src.ai_trainer.classes import DatabaseSAC, baseline_reward_function
from stable_baselines3.common.type_aliases import TrainFreq, TrainFrequencyUnit


class SACTests(unittest.TestCase):
    """Tests both variations of SAC whether they can work without an environment."""

    def setUp(self):
        """Set up the required variables for the tests."""
        self.env = gym.make("Pendulum-v0")
        self.rwf = baseline_reward_function
        self.rpbdefault = ReplayBuffer(1000, self.env.observation_space, self.env.action_space)
        self.rpbcustom = ReplayBuffer(1000, self.env.observation_space, self.env.action_space)
        self.default = SAC("MlpPolicy", self.env)
        self.custom = DatabaseSAC("MlpPolicy", self.rwf, self.env.observation_space, self.env.action_space)

    def test__store_transitions(self):
        """Test whether the algorithms store data properly."""
        A = np.array([-1])
        R = np.array([-1])
        Sn = np.array([0, 0, 0])
        d = np.array([0])
        i = [{"loss": 0}]
        self.default._store_transition(self.rpbdefault, A, Sn, R, d, i)
        self.custom._store_transition(self.rpbcustom, A, Sn, R, d, i)

    def test_collect_rollouts(self):
        """Test whether the algorithms call the collect_rollouts method properly."""
        # TODO: Navigate to pendulum.py (use debugger) and remove indexing on `u`.
        # TODO:
        A = np.array([-1])
        R = np.array([-1])
        Sn = np.array([0,0,0])
        d = np.array([0])
        i = [{"loss": 0}]
        self.default.env.reset()
        self.default._store_transition(self.rpbdefault, A, Sn, R, d, i)
        self.custom._store_transition(self.rpbcustom, A, Sn, R, d, i)
        self.default.collect_rollouts(self.default.env, CheckpointCallback(1000, "testoutputs/"),
                                      TrainFreq(1, TrainFrequencyUnit.STEP), self.rpbdefault)
        self.custom.collect_rollouts(self.custom.env, CheckpointCallback(1000, "testoutputs/"),
                                     TrainFreq(1, TrainFrequencyUnit.STEP), self.rpbcustom)
