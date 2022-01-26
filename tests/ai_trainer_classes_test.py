"""Contains tests to verify the main functionality of the custom classes found in ai_trainer/classes.py."""

import unittest
import numpy as np
import random
import gym

from gym.spaces import Discrete
from stable_baselines3.common.buffers import ReplayBuffer
from stable_baselines3.sac import SAC
from src.ai_trainer.classes import CustomReplayBuffer, CustomSAC


class ReplayBufferTests(unittest.TestCase):
    """Tests whether the custom replay buffer class follow the documented requirements for implementation."""

    def setUp(self):
        """Set up the required variables and objects for the tests."""
        self.S = [5, 8, 3]
        self.A = [1, 1, 2]
        self.R = [-1, -1, 5]
        self.Sn = [8, 3, 6]
        self.d = [0, 0, 1]
        self.i = [{"loss": 0}, {"loss": 0}, {"loss": 0}]  # Var useful for debugging, not so much for a test case.
        self.default = ReplayBuffer(1000, Discrete(10), Discrete(4))
        self.custom = CustomReplayBuffer(1000, Discrete(10), Discrete(4))

    def test_rbc_equality(self):
        """Test whether the replay buffer class satisfies the documented requirements."""
        # Add the same sample to each buffer.
        sample = random.randrange(0, 3)
        S = np.array(self.S[sample])
        Sn = np.array(self.Sn[sample])
        A = np.array(self.A[sample])
        R = np.array(self.R[sample])
        d = np.array(self.d[sample])
        i = [self.i[sample]]
        self.default.add(S, Sn, A, R, d, i)
        self.custom.add(S, Sn, A, R, d, i)
        # I did not find any methods to retrieve a specific sample from the replay buffers.
        # So we just use a single sample
        # Both replay buffers must be able to
        sample1 = self.default.sample(1)
        sample2 = self.custom.sample(1)

        self.assertEqual(sample1, sample2, "Both samples are not equal")


class SACTests(unittest.TestCase):
    """Tests both variations of SAC whether they can work without an environment."""

    def setUp(self):
        """Set up the required variables for the tests."""
        self.env = gym.make("Pendulum-v0")
        self.rpb = ReplayBuffer(1000, self.env.observation_space, self.env.action_space)
        self.default = SAC("MlpPolicy", self.env)
        self.custom = CustomSAC("MlpPolicy")

    def test_collect_rollouts(self):
        """Test whether the algorithms call the collect_rollouts method properly."""
        self.default.collect_rollouts(self.env, None, (5, "step"), self.rpb)
        self.custom.collect_rollouts(None, None, (5, "step"), self.rpb)

    def test__store_transitions(self):
        """Test whether the algorithms store data properly."""
        A = np.array([1])
        R = np.array([-1])
        Sn = np.array([8])
        d = np.array([0])
        i = [{"loss": 0}]
        self.default._store_transition(self.rpb, A, Sn, R, d, i)
        self.custom._store_transition(self.rpb, A, Sn, R, d, i)
