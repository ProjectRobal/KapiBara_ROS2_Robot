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

from sensor_msgs.msg import Imu,LaserScan
from geometry_msgs.msg import Twist


class Catch(gym.Env):
    metadata = {"render_modes": ["human"]}
    
    def robot_collison_callback(self,contacts:ContactsState):
        for contact in contacts.states:
            if contact.collision1_name.find("Maze") > -1:
                self._robot_has_hit_wall = True
                return
            if contact.collision2_name.find("Maze") > -1:
                self._robot_has_hit_wall = True
                return
            
    def mouse_tof_callback(self,tof_msg:LaserScan):
        range = min(tof_msg.ranges)
        
        if range > tof_msg.range_max:
            range = tof_msg.range_max
        
        self._mouse_distance = range
        
        self._node.get_logger().debug(f"Got range id: {id}")
            
    
    
    def __init__(self, render_mode=None,sequence_length=1,max_distance=4,mouse_speed = 0.25,timeout=30):

        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        # Inputs are 4 laser sensors distance, quanterion
        
        high = np.array([1.0,1.0,1.0,1.0 , 1.0,1.0,1.0,1.0 , *([1.0]*40*30*3) ]*sequence_length,dtype=np.float32)
        low = np.array([0.0,0.0,0.0,0.0 , -1.0,-1.0,-1.0,-1.0 , *([0.0]*40*30*3)]*sequence_length,dtype=np.float32)
        
        self._max_distance = max_distance
        
        self._timeout = timeout
        
        self._robot_has_hit_wall = False       
        self._mouse_distance = 10 
        self._distance_between = 0.0
        
        self._max_angular_speed = 2.0
        self._max_linear_speed = mouse_speed

        self.last_distance = 0
                
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

        self._mouse_contact_topic = self._node.create_subscription(ContactsState,"/KapiBara/collision",self.robot_collison_callback,10)
        self._mouse_tof_sensor = self._node.create_subscription(LaserScan,"/mouse/front",self.mouse_tof_callback,10)
        
        self._mouse_twist_output = self._node.create_publisher(Twist, "/mouse/mouse_motors/cmd_vel_unstamped", 10)
        
        # Start an Gazbo using proper launch file
        self._env = launch_environment("mouse.trap.one")
        self._env.start()
        
        # create client for step control service for KapiBara robot
        self._robot = KapiBaraStepAgent(self._node,max_linear_speed=2, max_angular_speed=4,position=[2.0,0.0,0.0],rotation=[0.0,0.0,0],reload_agent=False,use_camera=True)
        # create client for service to control gazebo environment
        
        self._sim = SimulationControl(self._node)
                
        self._sim.pause()
        self._sim.reset()
                
    def move_mouse(self,linear,angular):
        
        twist = Twist()
        
        linear = np.clip(int(linear),-1,1)
        angular = np.clip(int(angular),-1,1)
                
        twist.angular.z = angular * self._max_angular_speed
        twist.linear.x = linear * self._max_linear_speed
        
        self._mouse_twist_output.publish(twist)
        
    def _get_distance(self):
        agent_info = self._sim.get_entity_state("kapibara")
        mouse_info = self._sim.get_entity_state("stefan")
        
        self._distance_between = np.linalg.norm(agent_info[0] - mouse_info[0])
        
        self._node.get_logger().info("Distance between robots: "+str(self._distance_between))
        
        return self._distance_between
    
    def _get_obs(self):
        return self._robot_data

    def _get_info(self):
        # get information about distance between mouse and kapibara
        return {"distance":self._distance_between}
        
    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)
        
        # reset gazebo
        self._node.get_logger().info("Resetting simulation")
        self._sim.reset()
        self._node.get_logger().info("Resetting robot")
        self._robot.reset_agent()
        
        self._robot_data = np.zeros(self._robot_data.shape,dtype = np.float32)
        
        self._robot_has_hit_wall = False       
        self._mouse_distance = 10
        
        self._get_distance()

        observation = self._get_obs()
        info = self._get_info()
        
        self._timer = self._node.get_clock().now().to_msg().sec
        self._last_robot_positon = self._sim.get_entity_state("kapibara")[0]

        self._get_distance()

        self.last_distance = self._distance_between

        return observation, info
    
    def append_observations(self,observation):
        self._robot_data = np.roll(self._robot_data,-len(observation))    
        self._robot_data[-len(observation):] = observation[:]
    
    # that doesn't work as it should
    def step(self, action):
        # Map the action (element of {0,1,2,3}) to the direction we walk in
        
        
        self._robot.move(action)
        
        if self._mouse_distance < 0.5:
            self.move_mouse(0.0,-1.0)
        else:
            self.move_mouse(1.0,0.0)
        
        self._sim.unpause()
        # wait couple of steps
        self._robot.wait_for_steps()
        
        self._sim.pause()
        
        self._get_distance()
        
        observation = self._robot.get_observations()
        
        frame = self._robot.get_camera_frame()[-1]
       
        self.append_observations(np.concatenate((observation[:8],frame)))
        
        # An episode is done iff the agent has reached the target
        terminated = False
        done = False
        
        # We want agent to do as few steps as possible
        #
        # Each step will give reward 0
        # It will get 10.0 for reaching catching a target
        # It will get -1.0 for timing out or hiting a wall
        # It will get -10.0 when distance bettween robots is to big
        
        # default reward for every step
        reward = 0.0

        if self.last_distance - self._distance_between <= 0:
            self._node.get_logger().info("Robot moved away from target")
            reward = -5
        else:
            self._node.get_logger().info("Robot moved towards target")
            reward = 2
        
        if self._distance_between < 0.85:
            reward = 10.0
            done = True
            self._node.get_logger().info("Robot catched the target!")
            
        if self._distance_between > self._max_distance:
            reward = -20.0
            terminated = True
            self._node.get_logger().info("Robot has lost the target!")
            
        if self._node.get_clock().now().to_msg().sec - self._timer > self._timeout:
            terminated = True
            reward = -10.0
            self._node.get_logger().info("Simulation timed out!")
            
        if self._robot_has_hit_wall:
            terminated = True
            reward = -10.0
            self._node.get_logger().info("Robot hit the wall!")
    
        info = self._get_info()  

        self.last_distance = self._distance_between      
                
        return self._get_obs(), reward, terminated, done, info
    
    def render(self):
        return None
    
    def close(self):
        self._env.kill()
        self._env.join()
        self._env.close()