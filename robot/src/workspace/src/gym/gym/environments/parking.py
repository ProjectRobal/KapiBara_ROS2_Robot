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

from copy import copy

from timeit import default_timer as timer

class Parking(gym.Env):
    metadata = {"render_modes": ["human"]}
    
    stages = ["searching","parking"]

    REWARD_TYPE = Literal["normal","genetic"]
        
    def point_callback(self,contacts:ContactsState):
        for contact in contacts.states:
            if contact.collision1_name.find("kapibara") > -1:
                self._point_id_triggered = contact.collision2_name.split("::")[0]
                return
            if contact.collision2_name.find("kapibara") > -1:
                self._point_id_triggered = contact.collision1_name.split("::")[0]
                return
            
    def robot_collison_callback(self,contacts:ContactsState):
        for contact in contacts.states:
            if contact.collision1_name.find("Parking") > -1:
                self._robot_has_hit_wall = True
                return
            if contact.collision2_name.find("Parking") > -1:
                self._robot_has_hit_wall = True
                return
    
    def __init__(self,reward_type:Literal[REWARD_TYPE] = "normal", render_mode=None,sequence_length=1):

        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        # Inputs are 4 laser sensors distance, quanterion
        
        high = np.array([1.0,1.0,1.0,1.0 , 1.0,1.0,1.0,1.0  ]*sequence_length,dtype=np.float32)
        low = np.array([0.0,0.0,0.0,0.0 , -1.0,-1.0,-1.0,-1.0 ]*sequence_length,dtype=np.float32)
        
        
        print("High shape: ",high.shape)

        self.reward_type = reward_type

        self.got_to_spot = False
        
        self._stall_timer = timer()
        
        self._point_id_triggered = ""
        
        self._robot_has_hit_wall = False
                
        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        # We have 4 actions, corresponding to "right", "up", "left", "down"
        self.action_space = spaces.Discrete(4)

        """
        The following dictionary maps abstract actions from `self.action_space` to
        the direction we will walk in if that action is taken.
        I.e. 0 corresponds to "right", 1 to "up" etc.
        """
        self._action_to_direction = {
            0: np.array([1, 0]),
            1: np.array([0, 1]),
            2: np.array([-1, 0]),
            3: np.array([0, -1]),
        }

        self.render_mode = render_mode
        
        self._robot_data = np.zeros(high.shape,dtype = np.float32)
        
        self._stage_number = 0
        
        rclpy.init()

        self._node=Node("maze_env")
        
        # Start an Gazbo using proper launch file
        self._env = launch_environment("parking.one")
        self._env.start()
                
        self._point_topics = {}
        
        for i in range(2):
            self._node.get_logger().info(f"Created subscription for point{i}")
            self._point_topics["point"+str(i)]=self._node.create_subscription(ContactsState,"/trigger"+str(i),self.point_callback,10)
        
        self._contact_topic = self._node.create_subscription(ContactsState,"/KapiBara/collision",self.robot_collison_callback,10)
        # create client for step control service for KapiBara robot
        self._robot = KapiBaraStepAgent(self._node,position=[-0.2,0.0,0.0],rotation=[0.0,0.0,0],reload_agent=False,use_camera=False,max_linear_speed=0.5,max_angular_speed=2)
        # create client for service to control gazebo environment
        
        self._sim = SimulationControl(self._node)
        
        self._sim.pause()
        self._sim.reset()
    
    def _get_obs(self):
        return self._robot_data

    def _get_info(self):
        
        # get information abut stage:
        return {"stage":self.stages[self._stage_number]}
        
    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)
        
        # reset gazebo
        self._node.get_logger().info("Resetting simulation")
        self._sim.reset()
        self._node.get_logger().info("Resetting robot")
        self._robot.reset_agent()
        
        self._robot_data = np.zeros(self._robot_data.shape,dtype = np.float32)
        
        self._stage_number = 0
        self._point_id_triggered = ""
        self._robot_has_hit_wall = False
        self.got_to_spot = False

        del self._point_topics
     

        observation = self._get_obs()
        info = self._get_info()
        
        self._stall_timer = timer()

        return observation, info
    
    def append_observations(self,observation):
        self._robot_data = np.roll(self._robot_data,-len(observation))    
        self._robot_data[-len(observation):] = observation[:]
    
    def step(self, action):
        # Map the action (element of {0,1,2,3}) to the direction we walk in
        
        if not hasattr(self,"_point_topics"):
            self._point_topics = {}
            
            for i in range(2):
                self._node.get_logger().info(f"Created subscription for point{i}")
                self._point_topics["point"+str(i)]=self._node.create_subscription(ContactsState,"/trigger"+str(i),self.point_callback,10)
                
            rclpy.spin_once(self._node)
            self._point_id_triggered = ""
        
        
        self._robot.move(action)
        self._sim.unpause()
        # wait couple of steps
        self._robot.wait_for_steps()
        
        self._sim.pause()
        
        observation = self._robot.get_observations()
                        
        self.append_observations(observation[:8])
        
        #print(self._robot_data.shape)
        # We use `np.clip` to make sure we don't leave the grid
        
        # An episode is done iff the agent has reached the target
        terminated = False
        done = False
        
        # We want agent to do as few steps as possible
        #
        # Each step will give reward -0.04
        # Robot should stick to a wall so it will get -0.25 for moving away from it and 
        # It will get 1.0 for find proper parking spot
        # It will get -1.0 for hitting wall and environment is terminated
        # When robot park properly simulation finish and robot recive 1.0
        
                

        # if self._robot_data[11] < 0.1:
        #     reward = -0.5
        #     self._node.get_logger().info("Robot hits the wall, terminated!, back sensor!")
        #     terminated = True
                
        info = self._get_info()

        if self.reward_type == "normal":
            reward = -0.04
        else:
            reward = 0.0

        

        for distance in observation[0:4]:
            if distance < 0.1:
                if self.reward_type == "normal":
                    reward = -10.0
                else:
                    reward += -(timer() - self._stall_timer)/60.0
                self._node.get_logger().info(f"Robot hits the wall, terminated!, sensor id: {id}")
                terminated = True
                break
            
        if self._robot_has_hit_wall:
            if self.reward_type == "normal":
                reward = -10.0
            else:
                reward += -(timer() - self._stall_timer)/60.0
            self._node.get_logger().info("Robot hits the wall, terminated!")
            terminated = True
            
        if timer() - self._stall_timer > 60*10:
            terminated =  True
            reward += -20.0
            self._node.get_logger().info("Robot timed out!")
            self._stall_timer = timer()

        if len(self._point_id_triggered) == 0:

            

            # if any of side sensor register a too large distance it means it is far away from wall
            if min(observation[2],observation[3]) > 0.4:
                if self.reward_type == "normal":
                    reward = -1.0
                else:
                    reward += -0.05
                self._node.get_logger().info(f"Robot moved away from wall too much")

        else:
            dist = abs(observation[2] - observation[3])


            if not self.got_to_spot:
                self.got_to_spot = True
                if self.reward_type != "normal":
                    reward += 50.0

            if dist <= 0.1:
                if self.reward_type == "normal":
                    reward = 10.0
                else:
                    reward += 100.0
                self._node.get_logger().info("Robot has parked properly!")
                done = True
            
        # self._node.get_logger().info("Reward: "+str(reward))
        # self._node.get_logger().info("Observations: "+str(observation))
         
                
        return self._get_obs(), reward, terminated, done, info
    
    def render(self):
        return None
    
    def close(self):
        self._env.kill()
        self._env.join()
        self._env.close()