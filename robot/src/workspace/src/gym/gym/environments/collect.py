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

from copy import copy,deepcopy

class Collect(gym.Env):
    metadata = {"render_modes": ["human"]}

    REWARD_TYPE = Literal["normal","genetic"]
    
    point_positions = [
        np.array([-1.86847, 2.12847]),
        np.array([0.109086, -1.9246]),
        np.array([-3.04748, -3.18496]),
        np.array([-4.68853, 2.104755])
    ]
        
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
            if contact.collision1_name.find("Maze") > -1:
                self._robot_has_hit_wall = True
                return
            if contact.collision2_name.find("Maze") > -1:
                self._robot_has_hit_wall = True
                return
    
    def __init__(self, render_mode=None,sequence_length=1,reward_type:Literal[REWARD_TYPE] = 'normal',stall_time_sec=30):

        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        # Inputs are 4 laser sensors distance, quanterion

        self.reward_type = reward_type
        
        high = np.array([1.0,1.0,1.0,1.0 , 1.0,1.0,1.0,1.0 , *([1.0]*40*30*3) ]*sequence_length,dtype=np.float32)
        low = np.array([0.0,0.0,0.0,0.0 , -1.0,-1.0,-1.0,-1.0 , *([0.0]*40*30*3)]*sequence_length,dtype=np.float32)
        
        
        print("High shape: ",high.shape)
        
        self._stall_time_sec = stall_time_sec
        
        self._point_id_triggered = ""
        
        self._point_collected = 0
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
        self._last_robot_data = self._robot_data
        
        rclpy.init()

        self._node=Node("maze_env")
                
        # run spin in seaparated thread
        #self._spin_thread = Thread(target=rclpy.spin,args=(self._node,))
        #self._spin_thread.start()
        # Start an Gazbo using proper launch file
        self._env = launch_environment("collect.one",points_count=str(len(self.point_positions)))
        self._env.start()

        self.distance_to_closest_point = np.linalg.norm(self.point_positions[1])
        self.id_of_closed_point = 1

        self.current_point_positions = deepcopy(self.point_positions)
        
                
        self._point_topics = {}
        
        self._contact_topic = self._node.create_subscription(ContactsState,"/KapiBara/collision",self.robot_collison_callback,10)
        # create client for step control service for KapiBara robot
        self._robot = KapiBaraStepAgent(self._node,position=[0.0,0.0,0.0],rotation=[0.0,0.0,0],reload_agent=False,use_camera=True,max_linear_speed=0.5,max_angular_speed=2.5)
        # create client for service to control gazebo environment
        
        self._sim = SimulationControl(self._node)

        self._start = True
        
        self._env_timer = self._node.get_clock().now().to_msg().sec
        
        self._sim.pause()
        self._sim.reset()
        
    def spawn_points(self):
        points=""
        
        points+=f"{self.point_positions[0][0]} {self.point_positions[0][1]}"
        
        for point in self.point_positions[1:]:
            points+=f",{point[0]} {point[1]}"
            
        spawn_points = launch_environment("collect.spawn.blocks",points=points)
        spawn_points.start()
        
        spawn_points.join(timeout=40)

        if spawn_points.exitcode is None:
            return False
        
        return True
        
    def remove_points(self):
        for point in self._point_topics.keys():
            self._sim.remove_entity(point)
        
    def point_subscriptions(self):
        
        self._point_topics = {}
        for i in range(len(self.point_positions)):
            self._node.get_logger().info(f"Created subscription for point{i}")
            self._point_topics["point"+str(i)]=self._node.create_subscription(ContactsState,"/trigger"+str(i),self.point_callback,10)
    
    def _get_obs(self):
        return self._robot_data

    def _get_info(self):

        position = self._sim.get_entity_state("kapibara")[0]
        # position = self._robot_data[8:10]
        
        if len(position) < 2:
            return {}
        
        output = []
        
        for point in self.current_point_positions:
            output.append(np.linalg.norm(point - position[:2]))
        # get information distance from obstacltes:
        return {"distances_to_points":output}
        
    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)
        
        # reset gazebo
        self._node.get_logger().info("Resetting simulation")
        self._sim.reset()
        self._node.get_logger().info("Resetting robot")
        self._robot.reset_agent()

        
        
        #self.remove_points()
        
        self._robot_data = np.zeros(self._robot_data.shape,dtype = np.float32)
        self._point_id_triggered = ""
        self._robot_has_hit_wall = False
        self._point_collected = 0

        self.current_point_positions = deepcopy(self.point_positions)

        observation = self._get_obs()
        info = self._get_info()
        
        self._timer = self._node.get_clock().now().to_msg().sec
        
        self._env_timer = self._node.get_clock().now().to_msg().sec
        
        self._last_robot_positon = self._sim.get_entity_state("kapibara")[0]

        if self._start:

            self.spawn_points()
            self._start = False

        for id,point in enumerate(self.point_positions):
            name = "point"+str(id)
            
            self._sim.set_entity_position(name,[point[0],point[1],0.5])
            
        
        #self.spawn_points()
        
        del self._point_topics
        
        self.point_subscriptions()

        return observation, info
    
    def append_observations(self,observation):
        self._robot_data = np.roll(self._robot_data,-len(observation))    
        self._robot_data[-len(observation):] = observation[:]
    
    # that doesn't work as it should
    def step(self, action):
        # Map the action (element of {0,1,2,3}) to the direction we walk in        
        self._robot.move(action)
        self._sim.unpause()
        # wait couple of steps
        self._robot.wait_for_steps()
        
        self._sim.pause()
        
        observation = self._robot.get_observations()
        
        frame = self._robot.get_camera_frame()[-1]
        
        robot_position = self._sim.get_entity_state("kapibara")[0]
                
        self.append_observations(np.concatenate((observation[:8],frame)))
        
        #print(self._robot_data.shape)
        # We use `np.clip` to make sure we don't leave the grid
        
        # An episode is done iff the agent has reached the target
        terminated = False
        done = False
        
        # We want agent to do as few steps as possible
        #
        # Each step will give reward -0.04
        # It will get 1.0 for reaching it is goal, finding all points
        # It will get 0.5 for finding one point
        # When robot hits the wall it will get -0.5 reward and environment is terminated
        
        # default reward for every step
        if self.reward_type == 'normal':
            reward = -0.01
        else:
            reward = 0.0

        if self.reward_type == 'normal' and ( action == 0 or action == 1 ):
            reward = -0.1

            distances = self._get_info()["distances_to_points"]

            new_id_of_closet_point = min(range(len(distances)),key =lambda x: distances[x])

            if self.id_of_closed_point == new_id_of_closet_point:
                distance = distances[self.id_of_closed_point]

                if distance < self.distance_to_closest_point:
                    self._node.get_logger().info("Robot got closer to point")
                    reward = 0.25
                else:
                    self._node.get_logger().info("Robot got furhter from point") 

                self.distance_to_closest_point = distance

            else:
                self._node.get_logger().info("New closet points id: "+str(new_id_of_closet_point))
                self.id_of_closed_point = new_id_of_closet_point
                self.distance_to_closest_point = distances[self.id_of_closed_point]
        
        # check sensor data
        id = 0
        for distance in observation[0:4]:
            id+=1
            if distance < 0.1 or self._robot_has_hit_wall:
                if self.reward_type == "normal":
                    reward += -0.5
                else:
                    distances = self._get_info()["distances_to_points"]
                    
                    close_distance = np.min(distances)
                    
                    reward += -2*close_distance
                    
                self._node.get_logger().info(f"Robot hits the wall, terminated!, sensor id: {id}")
                terminated = True
                break
        
        if self._robot_has_hit_wall:
            if self.reward_type == "normal":
                    reward += -0.5
            else:
                distances = self._get_info()["distances_to_points"]
                    
                close_distance = np.min(distances)
                    
                reward += -2*close_distance
                
            self._node.get_logger().info("Robot hits the wall, terminated!")
            terminated = True
        
        info = self._get_info()
                
        if self._point_id_triggered in self._point_topics.keys():
            self._point_collected +=1
            if self.reward_type == "normal":
                reward = 5
            else:
                reward += 10
            self._node.get_logger().info("Robot found a point"+self._point_id_triggered)
                                    
            #self._sim.set_entity_position(self._point_id_triggered,np.array([-100.0*(self._point_collected+1),-100.0,0.4]))
            
            # self._sim.remove_entity(self._point_id_triggered)

            self._sim.set_entity_position(self._point_id_triggered,[10.0,10.0,0.0])
            
            del self._point_topics[self._point_id_triggered]

            del self.current_point_positions[self.id_of_closed_point]
            
            self._point_id_triggered = ""
        
        if np.linalg.norm(self._last_robot_positon - robot_position) <= 0.015:
            if self._node.get_clock().now().to_msg().sec - self._timer >= self._stall_time_sec:
                terminated = True
                self._node.get_logger().info("Robot has stalled!")
                reward += -10.0
                if self.reward_type != "normal":
                    reward += -10*( self._node.get_clock().now().to_msg().sec - self._env_timer )
        else:
            self._timer = self._node.get_clock().now().to_msg().sec
                
        self._last_robot_positon = robot_position
         
        if self._point_collected == len(self.point_positions):
            done = True
            if self.reward_type == "normal":
                reward = 10.0
            else:
                reward += 25.0 
                
        return self._get_obs(), reward, terminated, done, info
    
    def render(self):
        return None
    
    def close(self):
        self._env.kill()
        self._env.join()
        self._env.close()