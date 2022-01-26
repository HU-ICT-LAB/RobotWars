"""Contains the custom classes for the replay buffer and SAC algorithm to be applied within the pipeline."""

from stable_baselines3.common.buffers import ReplayBuffer
from stable_baselines3.sac.sac import SAC


def baseline_reward_function(S, A, Sn):
    """Return a reward based on the given parameters."""
    pass


class RewardFunction:
    """Reward function class that holds the method for assigning a reward to data."""

    def __init__(self, rewardfunc: callable):
        """Initialise the class."""
        self.rf = rewardfunc

    def add_reward(self, S, A, Sn):
        """Add a reward based based on the given parameters."""
        return self.rf(S, A, Sn)


class CustomReplayBuffer(ReplayBuffer):
    """Internal Replay Buffer that can work without requiring an environment."""

    pass


class CustomSAC(SAC):
    """Soft Actor Critic model that does not require an environment in order to run."""

    def collect_rollouts(self, rewardfunc: RewardFunction):
        """Process data from experience database into replay buffer."""
        pass

    def _store_transition(self, replaybuffer, action, nextobs, reward, done, info):
        """Handle storing the data from the environment into the given replay buffer."""
        pass
