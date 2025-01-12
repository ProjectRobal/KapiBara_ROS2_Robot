'''

 A module for controling KapiBara robot in step mode.
 
 It takes action to move in one of specified direction: 
    - left
    - right
    - top
    - bottom
    
 It also keep tracks on distance sensors, orientation and position in simulation
'''

import numpy as np

import base64

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu,LaserScan

from geometry_msgs.msg import Twist

from sensor_msgs.msg import CompressedImage

from nav_msgs.msg import Odometry

from rosgraph_msgs.msg import Clock

from gazebo_msgs.srv import DeleteEntity

from PIL import Image

from gym.utils.utils_launch import launch_other

import io

class KapiBaraStepAgent:
    
    '''
        Actions for agents: up , down , left , right
        in form of pair ( linear speed, angular speed)
    '''
    _actions = [(-1.0,0.0),(1.0,0.0),(0.0,1.0),(0.0,-1.0)]
    
    def clock_step_counter(self,clock):
        self._step_count += 1
        
        self._node.get_logger().debug(f"Got frame: {self._step_count}")
    
    def tof_callback(self,id,tof_msg:LaserScan):
        range = min(tof_msg.ranges)
        
        if range > tof_msg.range_max:
            range = tof_msg.range_max
        
        self._observations[id] = range
        
        self._node.get_logger().debug(f"Got range id: {id}")
        
    def orientaion_callback(self,imu:Imu):
        
        orientation = imu.orientation
        
        self._observations[4] = orientation.x
        self._observations[5] = orientation.y
        self._observations[6] = orientation.z
        self._observations[7] = orientation.w
        
        self._node.get_logger().debug("Got orientation!")
    
    def odometry_callback(self,odometry:Odometry):
        position = odometry.pose.pose.position
        
        self._observations[8] = position.x
        self._observations[9] = position.y
        self._observations[10] = position.z
        
        self._node.get_logger().debug("Got odometry!")
        
    def remove_agent(self):
        
        request = DeleteEntity.Request()
        
        request.name = "kapibara"
        
        future = self._remove_agent.call_async(request)
        
        while rclpy.ok():
            rclpy.spin_once(self._node)
            if future.done():
                break
            
    def camera_listener(self, msg:CompressedImage):
        self._node.get_logger().debug('I got image with format: %s' % msg.format)
        
        #image_buffer=(b"data:image/png;base64,"+base64.b64encode())
        
        image = Image.open(io.BytesIO(msg.data))
        
        image = image.resize((40,30)).convert(mode="RGB")
        
        last_frame = np.asarray(image).astype(np.float32)/256.0
        
        if len(self.frames)>10:
            del self.frames[0]
        
        self.frames.append(last_frame.reshape(40*30*3))
    
    def init_camera(self):
        
        self.frames = []
        
        self.subscription = self._node.create_subscription(
            CompressedImage,
            "/KapiBara/camera/image_raw/compressed",
            self.camera_listener,
            10)
        
    def get_camera_frame(self):
        
        if len(self.frames) == 0:
            return [np.zeros(40*30*3)]
        
        return self.frames
    
    def __init__(self,parent_node:Node, max_linear_speed:float=None, max_angular_speed:float=None,position = [0.0]*3,rotation = [0.0]*3,reload_agent=True,use_camera=False) -> None:
        
        # agent default positon and rotation
        self.position = np.array(position).astype(np.float32)
        self.rotation = np.array(rotation).astype(np.float32)
        
        self.reload_agent = reload_agent
        self._robot_spawned = False
        
        self._node = parent_node
        
        self._observations = np.zeros(11,dtype=np.float32)
        
        if max_angular_speed is None:
            self._max_angular_speed = 1.0
        else:
            self._max_angular_speed = max_angular_speed
            
        if max_linear_speed is None:
            self._max_linear_speed = 0.25
        else:
            self._max_linear_speed = max_linear_speed
            
            
        # create service client for agent removing        

        self._remove_agent = self._node.create_client(DeleteEntity,"/delete_entity")
        
        # wait 60 seconds for service ready
        if not self._remove_agent.wait_for_service(60):
            raise TimeoutError("Cannot connect to service: /delete_entity")
        
        # creates subscription for laser sensors
        
        self._tof_sub = []
        
        self._tof_sub.append(self._node.create_subscription(LaserScan,"/KapiBara/front_left",lambda msg: self.tof_callback(0,msg),10))
        self._tof_sub.append(self._node.create_subscription(LaserScan,"/KapiBara/front_right",lambda msg: self.tof_callback(1,msg),10))
        self._tof_sub.append(self._node.create_subscription(LaserScan,"/KapiBara/side_left",lambda msg: self.tof_callback(2,msg),10))
        self._tof_sub.append(self._node.create_subscription(LaserScan,"/KapiBara/side_right",lambda msg: self.tof_callback(3,msg),10))
        # self._tof_sub.append(self._node.create_subscription(LaserScan,"/KapiBara/floor",lambda msg: self.tof_callback(11,msg),10))
                
        # creates subscription for orientation
        
        self._orientaion_sub = self._node.create_subscription(Imu,"/KapiBara/orientation",self.orientaion_callback,10)
        
        # creates subcription for position
        
        self._odometry_sub = self._node.create_subscription(Odometry,"/KapiBara/motors/odom",self.odometry_callback,10)
        
        # creates publisher for ros2 control twist
        
        self.twist_output = self._node.create_publisher(Twist, "/KapiBara/motors/cmd_vel_unstamped", 10)
        
        self._step_count = 0
        
        self._step_counter_qos =  rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                                          depth=10)
        
        # step counter
        self._step_counter = self._node.create_subscription(Clock,"/clock",self.clock_step_counter,qos_profile=self._step_counter_qos)
        
        if use_camera:
            self.init_camera()

        
    def move(self,direction):
        
        twist = Twist()
        
        direction = np.clip(direction,0,3)
        
        action = self._actions[direction]
        
        twist.angular.z = action[1] * self._max_angular_speed
        twist.linear.x = action[0] * self._max_linear_speed
        
        self.twist_output.publish(twist)
                
        #rclpy.spin_once(self._node)
        
    def reset_agent(self):
        '''
            Remove agent entity from gazebo and spawn it once again.
        '''
        
        if not self.reload_agent and self._robot_spawned:
            return
            
        self.remove_agent()
        
        reseted_succesfuly=False
        
        while not reseted_succesfuly:
            self._node.get_logger().info("Attempting to reset robot")
        
            process = launch_other("spawn.robot",x=str(self.position[0]),y=str(self.position[1]),z=str(self.position[2]),roll=str(self.rotation[0]),pitch=str(self.rotation[1]),yaw=str(self.rotation[2]))
            
            process.start()
            
            process.join(timeout=3.0)
            
            if process.exitcode is None or process.exitcode != 0:
                self._node.get_logger().info("Failed to reset robot, trying again!")
                process.terminate()
            else:
                reseted_succesfuly = True
                self._node.get_logger().info("Robot restarted succesfully")
                self._robot_spawned = True
        
        
    def wait_for_steps(self):
        self._step_count = 0
        while self._step_count < 10:
            self._node.get_logger().debug("Waiting for clock!")
            rclpy.spin_once(self._node)
            
    def get_observations(self)->np.ndarray:
        
        rclpy.spin_once(self._node)
        
        return self._observations