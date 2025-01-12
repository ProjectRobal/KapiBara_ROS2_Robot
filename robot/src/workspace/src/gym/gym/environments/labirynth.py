from typing import Literal
import numpy as np

import gymnasium as gym
from gymnasium import spaces

from gym.utils.utils_launch import launch_environment

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ContactsState

from gym.agents.step_agnet import KapiBaraStepAgent
from gym.utils.utils_sim import SimulationControl

from threading import Thread

class Labirynth(gym.Env):
    metadata = {"render_modes": ["human"]}
    
    MAZE_LIST = Literal["basic","normal"]
    REWARD_TYPE = Literal["normal","distance"]
    
    def trigger0_callback(self,contacts:ContactsState):
        for contact in contacts.states:
            if contact.collision1_name.find("kapibara") > -1 or contact.collision2_name.find("kapibara") > -1:
                self._trigger0_triggered = True
                
    def robot_collison_callback(self,contacts:ContactsState):
        for contact in contacts.states:
            if contact.collision1_name.find("Maze") > -1:
                self._robot_has_hit_wall = True
                return
            if contact.collision2_name.find("Maze") > -1:
                self._robot_has_hit_wall = True
                return
    
    def __init__(self, render_mode=None , maze: Literal[MAZE_LIST] = 'basic',reward_type: Literal[REWARD_TYPE] = 'normal',sequence_length=1,stall_time_sec=50):

        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        # Inputs are 4 laser sensors distance, quanterion , position
        
        high = np.array([1.0,1.0,1.0,1.0 , 1.0,1.0,1.0,1.0 , np.inf,np.inf,np.inf ]*sequence_length,dtype=np.float32)
        low = np.array([0.0,0.0,0.0,0.0 , -1.0,-1.0,-1.0,-1.0 , -np.inf,-np.inf,-np.inf ]*sequence_length,dtype=np.float32)
        
        self._trigger0_triggered = False
        self._robot_has_hit_wall = False
        self._stall_time_sec = stall_time_sec
      
        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        # We have 4 actions, corresponding to "right", "up", "left"
        self.action_space = spaces.Discrete(3)
        
        self.reward_type = reward_type

        
        self._action_to_direction = {
            0: np.array([1, 0]),
            1: np.array([0, 1]),
            # 2: np.array([-1, 0]),
            2: np.array([0, -1]),
        }

        self.render_mode = render_mode
        
        self._robot_data = np.zeros(high.shape,dtype = np.float32)        
        rclpy.init()

        self._node=Node("maze_env")
        
        # Start an Gazbo using proper launch file
        # since target is immovable the position is constant for entire simulation

        if maze == 'basic':
            self._target_data = np.array([-2.3314,2.4931,0.4],dtype=np.float32)
            self._env = launch_environment("labirynth")
        elif maze == 'normal':
            self._env = launch_environment("labirynth.big")
            self._target_data = np.array([-2.761,3.045,0.4],dtype=np.float32)
        self._env.start()
        
        self._trigger0_topic = self._node.create_subscription(ContactsState,"/trigger0",self.trigger0_callback,10)
        self._contact_topic = self._node.create_subscription(ContactsState,"/KapiBara/collision",self.robot_collison_callback,10)
                
        # create client for step control service for KapiBara robot
        if maze == 'basic':
            self._robot = KapiBaraStepAgent(self._node,position=[0.0,0.20,0.0],rotation=[0.0,0.0,-1.57],reload_agent=False)
        elif maze == 'normal':
            self._robot = KapiBaraStepAgent(self._node,position=[0.0,0.035,0.0],rotation=[0.0,0.0,0],reload_agent=False)
        # create client for service to control gazebo environment
        
        self._sim = SimulationControl(self._node)
        
        self._sim.pause()
        self._sim.reset()
    
    def _get_obs(self):
        return self._robot_data

    
    def _get_info(self):
        return {
            "distance": np.linalg.norm(
                self._robot_data[-3:] - self._target_data, ord=1
            )
        }
        
    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)
        
        # reset gazebo
        self._node.get_logger().info("Resetting simulation")
        self._sim.reset()
        self._node.get_logger().info("Resetting robot")
        self._robot.reset_agent()
        
        self._robot_data = np.zeros(self._robot_data.shape,dtype = np.float32)
        self._trigger0_triggered = False
        self._robot_has_hit_wall = False
        
        self._timer = self._node.get_clock().now().to_msg().sec
        
        self._last_robot_positon = self._sim.get_entity_state("kapibara")[0]

        observation = self._get_obs()
        info = self._get_info()

        return observation, info
    
    def append_observations(self,observation):
        self._robot_data = np.roll(self._robot_data,-len(observation))    
        self._robot_data[-len(observation):] = observation[:]
    
    # that doesn't work as it should
    def step(self, action):
        # Map the action (element of {0,1,2,3}) to the direction we walk in
        
        if action >=1:
            action += 1
        
        self._robot.move(action)
        self._sim.unpause()
        # wait couple of steps
        self._robot.wait_for_steps()
        
        self._sim.pause()
        
        robot_position = self._sim.get_entity_state("kapibara")[0]
        
        observation = self._robot.get_observations()
        
        observation[8:11] = robot_position[:]
        
                
        self.append_observations(observation)
        # We use `np.clip` to make sure we don't leave the grid
        
        # An episode is done iff the agent has reached the target
        terminated = False
        
        r_distance = self._get_info()['distance']
        
        # self._node.get_logger().info(f"Robot distance: {r_distance}")

        
        # We want agent to do as few steps as possible
        #
        # Each step will give reward -0.04
        # Moving backwards will give -0.25
        # It will get 1.0 for reaching it is goal, hiting a sqaure
        # When robot is too near a wall it will get -0.25
        # Robot gets -10.0 reward for staying in the same place
        #
        # When robot hits the wall the environment is terminated
        
        # default reward for every step
        if self.reward_type == "normal":
            reward = -0.04
        else:
            reward = 0.0
        
        # check sensor data
        id = 0
        for distance in observation[0:4]:
            id+=1
            if distance < 0.1:
                if self.reward_type == "normal":
                    reward = -1.0
                else:
                    reward = -r_distance
                self._node.get_logger().info(f"Robot hits the wall, terminated!, sensor id: {id}")
                terminated = True
                break
            # elif distance < 0.35:
            #     reward = -0.5
            #     self._node.get_logger().info(f"Robot sticks to wall!, sensor id: {id}")
            #     break
        
        if action == 1:
            self._node.get_logger().info("Robot moved backward!")
            reward = -0.25
            
        if self._robot_has_hit_wall:
            if self.reward_type == "normal":
                reward = -1.0
            else:
                reward = -r_distance
            self._node.get_logger().info("Robot hits the wall, terminated!")
            terminated = True
            
        info = self._get_info()

        # check if robot reached end of the maze
        if self._trigger0_triggered:
            done = True
            self._node.get_logger().info("Robot has found the target!")
            self._trigger0_triggered = False
            reward = 1000.0
        else:
            done = False
            
        d_distance = np.linalg.norm(self._last_robot_positon - robot_position) 
        self._node.get_logger().debug(f"Robot moved by {d_distance}")
        
        if np.linalg.norm(self._last_robot_positon - robot_position) <= 0.015:
            if self._node.get_clock().now().to_msg().sec - self._timer >= self._stall_time_sec:
                terminated = True
                self._node.get_logger().info("Robot has stalled!")
                if self.reward_type == "normal":
                    reward = -10.0
                else:
                    reward = -r_distance
        else:
            self._timer = self._node.get_clock().now().to_msg().sec
                
        self._last_robot_positon = robot_position

        return self._get_obs(), reward, terminated, done, info
    
    def render(self):
        return None
    
    def close(self):
        self._env.kill()
        self._env.join()
        self._env.close()