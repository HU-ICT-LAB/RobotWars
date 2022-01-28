"""Contains the custom classes for the SAC algorithm to be applied within the pipeline."""


from stable_baselines3.common.buffers import ReplayBuffer
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.noise import ActionNoise
from stable_baselines3.common.type_aliases import RolloutReturn, TrainFreq, TrainFrequencyUnit

import random

from stable_baselines3.common.vec_env import VecEnv, DummyVecEnv
from stable_baselines3.sac.sac import SAC, SACPolicy

from typing import Union, Type, Callable, List, Tuple, Dict, Optional
from dataclasses import dataclass

import datetime

import numpy as np

import gym


@dataclass
class Transition:
    """Dataclass that holds information about transitions."""

    timestamp: datetime
    observation: np.array
    action: np.array
    robot_id: int
    reward = 0


@dataclass
class Event:
    """Dataclass that holds information about events."""

    timestamp: datetime
    event_type: str
    subject: int


def find_closest_transitions(transitions: List[Transition], timestamp: datetime) -> Dict[int, Transition]:
    """Find closest transition for each robot based on a list of transitions and a given timestamp."""
    returndict = dict()
    datetimes = [transition.timestamp for transition in transitions]
    robotids = [transition.robot_id for transition in transitions]
    deltas = [dt - timestamp for dt in datetimes]
    for robotid in robotids:
        relevanttrans = [transitions[i] for i in range(len(datetimes)) if robotids[i] == robotid]
        relevantdeltas = [deltas[i] for i in range(len(datetimes)) if robotids[i] == robotid]
        closestindex = relevantdeltas.index(min(relevantdeltas))
        returndict[robotid] = relevanttrans[closestindex]
    return returndict


def baseline_reward_function(transitions: List[Transition], events: List[Event]) -> None:
    """Return a reward based on the given parameters."""
    for event in events:
        closest_transitions = find_closest_transitions(transitions, event.timestamp)
        for transition in closest_transitions:
            if event.event_type == 'hit':
                if transition.robot_id == event.subject:
                    transition.reward -= 10
                else:
                    transition.reward += 10
            if event.event_type == 'shoot':
                if transition.robot_id == event.subject:
                    transition.reward -= 1
            if event.event_type == 'collision':
                if transition.robot_id == event.subject:
                    transition.reward -= 10


def get_database_transitions_size():
    """Return the total amount of transitions registered in the database."""
    raise NotImplementedError


def get_database_transitions(transitions_ids: List[int]):
    """Return transitions from the database matching an entry in transitions_ids."""
    raise NotImplementedError


def get_database_events(between_range: Tuple[datetime.datetime, datetime.datetime]):
    """Return events from the database where the timestamp is within the given range."""
    raise NotImplementedError


class DatabaseSAC(SAC):
    """Soft Actor Critic model that does not require an environment in order to run."""

    def __init__(self, policy: Union[str, Type[SACPolicy]],
                 reward_function: Callable[[List[Transition], List[Event]], None],
                 observation_space: gym.spaces.Space, action_space: gym.spaces.Space):
        """Reward function should be an inline function that adds the reward to the transitions based on the given events."""
        super().__init__(policy, DummyVecEnv([lambda: gym.make("Pendulum-v0")]))
        self.reward_function = reward_function
        self.gotten_transition_ids = []

    def collect_rollouts(
            self,
            env: VecEnv,
            callback: BaseCallback,
            train_freq: TrainFreq,
            replay_buffer: ReplayBuffer,
            action_noise: Optional[ActionNoise] = None,
            learning_starts: int = 0,
            log_interval: Optional[int] = None,
    ) -> RolloutReturn:
        """Process data from experience database into replay buffer."""
        # TODO: Request experience data from the database using SQL
        # TODO: SQL query: Sort by date, reverse, only take as many transitions
        callback.on_rollout_start()
        num_collected_steps, num_collected_episodes = 0, 0

        assert train_freq.unit == TrainFrequencyUnit.STEP, "DatabaseSAC only supports training in steps, not episodes."

        wanted_number_of_transitions = train_freq.frequency
        n_transition_in_db = get_database_transitions_size()
        transitions_ids = []

        while len(transitions_ids) < wanted_number_of_transitions:
            random_id = random.randrange(0, n_transition_in_db)
            if random_id not in self.gotten_transition_ids:
                transitions_ids.append(random_id)
                self.gotten_transition_ids.append(random_id)

        transitions = get_database_transitions(transitions_ids=transitions_ids)
        earliest_transition = min(t.timestamp for t in transitions)
        latest_transition = max(t.timestamp for t in transitions)

        events = get_database_events(between_range=(earliest_transition.timestamp, latest_transition.timestamp))

        self.reward_function(transitions, events)

        sortedtransitions = sorted(transitions, key=lambda t: datetime.datetime.strptime(t.timestamp, '%Y/%m/%d %H:%M:%S'))
        # TODO: Format must be edited to match the format used in the database.

        for j in range(len(sortedtransitions)):
            self._store_transition(replay_buffer, transitions[j].action, transitions[j + 1].observation,
                                   transitions[j].reward, np.array([0]), [{"robotid": transitions[j].robotid}])
            callback.update_locals(locals())
            if callback.on_step() is False:
                return RolloutReturn(0.0, episode_timesteps=num_collected_steps * env.num_envs,
                                     n_episodes=num_collected_episodes, continue_training=False)

        callback.on_rollout_end()

        return RolloutReturn(0.0, episode_timesteps=num_collected_steps * env.num_envs,
                             n_episodes=num_collected_episodes, continue_training=True)
