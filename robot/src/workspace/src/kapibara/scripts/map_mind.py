#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Range,Imu,PointCloud2
from geometry_msgs.msg import Quaternion

from std_msgs.msg import Float64MultiArray

from kapibara_interfaces.msg import Emotions
from kapibara_interfaces.msg import Microphone
from kapibara_interfaces.msg import PiezoSense

from sensor_msgs.msg import CompressedImage,Image
from sensor_msgs.msg import PointCloud2,PointField

from sensor_msgs.msg import CompressedImage,Image
from sensor_msgs.msg import PointCloud2,PointField

from rcl_interfaces.msg import ParameterDescriptor

from kapibara_interfaces.msg import FaceEmbed

from kapibara_interfaces.srv import StopMind

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import webrtcvad

import cv2
from cv_bridge import CvBridge

import numpy as np

from timeit import default_timer as timer
import time

from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance,OrderBy,PayloadSchemaType
from qdrant_client.http import models

import random

ID_TO_EMOTION_NAME = [
    "angry",
    "fear",
    "happiness",
    "uncertainty",
    "boredom"
    ]

IMG_EMBEDDING_SHAPE = 32*32*4
IMG_ACTION_SET_NAME = "actions_sets"

def generate_embedding(img,depth):
    '''
    generate_embedding 
    
    In that case it takes image and 
    depth image, merges them and split them
    into equal parts.
    
    :param img: Associated RGB image
    :param depth: Associated depth image
    '''
    
    img = cv2.resize(img,(224,224)).astype(np.float32) / 255.0
    depth = cv2.resize(depth,(224,224))
    
    img_merged = cv2.merge([img,depth])
    
    x = 0
    y = 0
    
    steps = int(224/32)
    
    embeddings = []
    
    for y in range(steps):
        for x in range(steps):
            embeddings.append(
                img_merged[x*32:x*32 + 32,y*32:y*32 + 32].flatten()
            )
        
    return embeddings


FACE_TOLERANCE = 0.94


class FaceSqlite:
    
    def __init__(self,db) -> None:
        
        self.db = db
        
        # self.db.delete_collection(
        #     collection_name="faces"
        # )
        
        self.init_databse()
        
    def init_databse(self):
        
        if not self.db.collection_exists("faces"):
            self.db.create_collection(
                collection_name="faces",
                vectors_config=VectorParams(size=160, distance=Distance.COSINE),
            )
            
            self.db.create_payload_index(
                collection_name="faces",
                field_name="time",
                field_schema=PayloadSchemaType.INTEGER
            )
            
            self.db.create_payload_index(
                collection_name="faces",
                field_name="score",
                field_schema=PayloadSchemaType.INTEGER
            )
    
    def get_face(self,face_embedding:np.ndarray):
        
        hits = self.db.query_points(
            collection_name="faces",
            query=face_embedding,
            limit=3
        )
        
        for hit in hits.points:
            if hit.score < 0.1:
                return hit.payload
            
    def check_size(self):
        
        return self.db.count("faces").count
        
    def update_face(self,face_embedding:np.ndarray,score:float):
        
        # check size
        
        size = self.check_size()
        
        if size >= 500:
            
            results = self.db.scroll(
                collection_name="faces",
                limit=1,
                order_by=OrderBy(
                    key="time",
                    direction="asc"
                )
            )
                        
            results = results[0][0]
                                                
            id = results.id
            
            payload = results.payload
            
            payload["score"] = score
            payload["time"] = time.time_ns()
            
            self.db.upsert(
                collection_name="faces",
                points=[
                    models.PointStruct(
                        id=id,
                        vector=face_embedding,
                        payload=payload  # new payload
                    )
                ]
            )
            
            return
            
        
        # check if face exists
        face = self.get_face(face_embedding)
        
        id = size+1
        
        if face is not None:
            id = face.id
                        
        payload = {}
        
        payload["score"] = score
        payload["time"] = time.time_ns()
        
        self.db.upsert(
            collection_name="faces",
            points=[
                models.PointStruct(
                    id=size+1,
                    vector=face_embedding,
                    payload=payload  # new payload
                )
            ]
        )



class MapMind(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('tick_time', 0.05)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.emotion_pub = self.create_publisher(Emotions,'emotions', 10)
        
        self.spectogram_publisher = self.create_publisher(Image, 'spectogram', 10)
        
        timer_period = self.get_parameter('tick_time').get_parameter_value().double_value  # seconds
        self.timer = self.create_timer(timer_period, self.tick_callback)
        
        self.image_sub = self.create_subscription(
            Image,
            'camera',
            self.image_callback,
            10)
        
        self.image_sub = self.create_subscription(
            Image,
            'depth',
            self.depth_image_callback,
            10)
        
        self.points_sub = self.create_subscription(
            PointCloud2,
            'points',
            self.points_callback,
            10)
        
        self.ext_emotion_sub = self.create_subscription(
            Emotions,
            'ext_emotion',
            self.ext_emotion_callback,
            10)
        
        self.mic_subscripe = self.create_subscription(Microphone,'mic',self.mic_callback,10)
                
        self.bridge = CvBridge()
        
        self.db = QdrantClient(host='0.0.0.0',port=6333)
        
        self.initialize_db()
        
        self.face_db = FaceSqlite(self.db)
                
        self.emotion_state = 0.0
        
        self.last_img_embeddings = np.zeros(IMG_EMBEDDING_SHAPE,dtype=np.float32)
        
        self.action_list = []
        self.action_executing = False
        self.action_iter = iter(self.action_list)
        
        self.image = None
        self.depth = None
        
        self.emotion = Emotions()
        
        # emotion state from external sources
        self.ext_emotion = Emotions()
        
        self.obstacle_detected = False
        
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)
        
        self.audio_fear = 0
        
        self.mic_buffor = np.zeros(2*16000,dtype=np.float32)
        
        self.wait_for_img = True
        self.wait_for_depth = True
        
        # angry
        # fear
        # happiness
        # uncertainty
        # boredom 
        # anguler values for each emotions state
        self.emotions_angle=[0.0,180.0,25.0,145.0,90.0] 
        
        self.ears_publisher = self.create_publisher(
            Float64MultiArray,
            'ears_controller/commands', 
            10
            )
        
    def initialize_db(self):
                        
        if not self.db.collection_exists(IMG_ACTION_SET_NAME):
            self.get_logger().warning("Image action set database not exist, creating new one!")
            self.db.create_collection(
                collection_name=IMG_ACTION_SET_NAME,
                vectors_config=VectorParams(size=IMG_EMBEDDING_SHAPE, distance=Distance.EUCLID),
            )
        
        
    
    def send_command(self,linear:float,angular:float):
        
        twist = Twist()
        
        linear = min(linear,1.0)
        linear = max(linear,-1.0)
        
        angular = min(angular,1.0)
        angular = max(angular,-1.0)
        
        twist.linear.x = linear*self.max_linear_speed
        twist.angular.z = angular*self.max_angular_speed
        
        self.twist_pub.publish(twist)
        
    def stop(self):
        self.send_command(0.0,0.0)
        
    def mic_callback(self,mic:Microphone):
        
        self.get_logger().debug("Mic callback")
        # I have to think about it
        
        self.mic_buffor = np.roll(self.mic_buffor,mic.buffor_size)
        
        left = np.array(mic.channel1,dtype=np.float32)/np.iinfo(np.int32).max
        right = np.array(mic.channel2,dtype=np.float32)/np.iinfo(np.int32).max
        
        combine = ( left + right ) / 2.0
        
        self.mic_buffor[:mic.buffor_size] = combine[:]
        
        start = timer()
        
        # speech_detect = self.vad.is_speech(self.mic_buffor, 16000)
        
        # if speech_detect:
        #     self.uncertain_speech = 1.0
        # We are going to replace it with something simpler
        # output,spectogram = self.hearing.input(self.mic_buffor)
        
        nperseg = 255
        noverlap = nperseg - 128 # 255 - 128 = 127

        # Calculate the spectrogram
        # f: array of sample frequencies
        # t_spec: array of segment times
        # Sxx: Spectrogram of x. By default, the last axis of Sxx corresponds to the segment times.
        # f, t_spec, _spectogram = spectrogram(self.mic_buffor, fs=16000, nperseg=nperseg, noverlap=noverlap)
                
        # _spectogram = cv2.normalize(_spectogram,None,alpha=0,beta=255,norm_type=cv2.NORM_MINMAX).astype(np.uint8)
        
        # publish last spectogram
        # self.spectogram_publisher.publish(self.bridge.cv2_to_imgmsg(_spectogram))
                
        self.get_logger().debug("Hearing time: "+str(timer() - start)+" s")
        
        # self.get_logger().debug("Hearing output: "+str(self.hearing.answers[output]))
        
        # self.audio_output = output
        
        mean = np.mean(np.abs(combine))
        
        if mean >= 0.7:
            self.audio_fear = 1.0
            
    def ext_emotion_callback(self,msg:Emotions):
        self.ext_emotion = msg
                
    def depth_image_callback(self,msg:Image):
        
        self.get_logger().info('I got depth image with format: %s' % msg.encoding)
        
        self.depth = self.bridge.imgmsg_to_cv2(msg)
        
        self.wait_for_depth = False
        
    def image_callback(self,msg:Image):
        
        self.get_logger().info('I got image with format: %s' % msg.encoding)
        
        self.image = self.bridge.imgmsg_to_cv2(msg)
        
        self.wait_for_img = False
                
    def points_callback(self, msg: PointCloud2):
        # Read points from PointCloud2
        points = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
            reshape_organized_cloud=True
        )
        
        sorted_points = np.sort(points)
        
        min_points = np.mean(sorted_points[0:10])
        
        self.obstacle_detected = min_points < 0.0
        
        if self.obstacle_detected:    
            self.get_logger().info(f'Obstacle detected!')
            
    def emotion_state_calculate(self,emotions:list[float]):
                
        return  emotions[2]*320.0 + emotions[1]*-120.0 + emotions[3]*-40.0 + emotions[0]*-60.0 + emotions[4]*-20.0
        
    def send_ears_state(self,emotions:list[float]):
        
        max_id = 4
        
        if np.sum(emotions) >= 0.01:
            max_id = np.argmax(emotions[:4])
            
        self.get_logger().debug("Current emotion: "+str(ID_TO_EMOTION_NAME[max_id]))
            
        self.get_logger().debug("Sending angle to ears: "+str(self.emotions_angle[max_id]))
        
        angle:float = (self.emotions_angle[max_id]/180.0)*np.pi
        
        array=Float64MultiArray()
        
        array.data=[np.pi - angle, angle]
        
        self.ears_publisher.publish(array)
        
    
    def check_actions_payload(self,payload:dict):
        
        return "actions" in payload.keys() \
                and "score" in payload.keys() \
                and type(payload["actions"]) is list \
                and type(payload["score"]) is float
                
    def attach_actions_to_image_embeddings(self,actions:list[tuple],score:float):
        
        new_id = self.db.count(IMG_ACTION_SET_NAME).count + 1
        
        for img_embedding in self.last_img_embeddings:
            self.db.upsert(
                collection_name=IMG_ACTION_SET_NAME,
                points=[
                        models.PointStruct(
                            id = new_id,
                            vector=img_embedding,
                            payload = {
                                    "score": score,
                                    "actions": actions
                                }
                            )
                    ]
            )
    
    def update_actions_for_embeddings(self,actions:list[tuple],score:float):
        
        self.attach_actions_to_image_embeddings(actions,score)
    
    def image_action_retrival(self):
        
        actions_lists = []
        
        if self.image is not None and self.depth is not None:
                    
            self.last_img_embeddings = generate_embedding(self.image,self.depth)
            
            # get actions lists associated with it
            
            # payloads for analazying
            
            for img_embedding in self.last_img_embeddings:
            
                hits = self.db.query_points(
                    collection_name=IMG_ACTION_SET_NAME,
                    query=img_embedding,
                    limit=3
                )
            
                points = hits.points
                
                for hit in points:
                    if hit.score < 0.25:
                        if self.check_actions_payload(hit.payload):
                            actions_lists.append(hit.payload)
            
                # self.get_logger().info(f'Points: {points}')
        else:
            self.get_logger().warning('No visual data!')
            
        return actions_lists
    
    def action_crossovers(self,actions_lists):
        
        actions = []
        
        if len(actions_lists) < 2:
            
            action_count = int(np.random.uniform(1,64))
            
            for i in range(action_count):
                
                v = np.random.random()
                w = np.random.random()
                
                actions.append((v,w))
                
            return actions
        
        # for now we will only do this with two actions
        # when we incorporate map we will use more predictive 
        # retrival approach
        
        actions.extend(actions_lists[0]["actions"])
        actions.extend(actions_lists[1]["actions"])
        
        # drop out
        
        to_remove = []
        
        if len(actions) > 64:
            action_mask = np.random.random(len(actions))
            for i in range(len(actions)):
                if action_mask[i] < 0.1:
                    to_remove.append(i)
                    
        # for to_rem in to_remove:
        #     actions.pop(to_rem)
                    
        action_mask = np.random.random(len(actions))
        
        # perform mutations
        
        for i in range(len(actions)):
            if action_mask[i] < 0.1:
                v = actions[i][0] + np.random.random()*0.1
                w = actions[i][1] + np.random.random()*0.1
                
                actions[i] = (v,w)
                    
        random.shuffle(actions)
        
        return actions
        
        
        
    
    def tick_func(self):
        # emotion validation pipeline
        
        # face detection
        
        # evaluate emotion states
        
        emotions = Emotions()
        
        emotions_arr = [
            self.emotion.angry + self.ext_emotion.angry,
            self.emotion.fear + self.ext_emotion.fear,
            self.emotion.happiness + self.ext_emotion.happiness,
            self.emotion.uncertainty + self.ext_emotion.uncertainty,
            self.emotion.boredom + self.ext_emotion.boredom
        ]
        
        emotions.angry = emotions_arr[0]
        emotions.fear = emotions_arr[1]
        emotions.happiness = emotions_arr[2]
        emotions.uncertainty = emotions_arr[3]
        emotions.boredom = emotions_arr[4]
        
        self.emotion_pub.publish(emotions)
        
        score = self.emotion_state_calculate(emotions_arr)
        
        # send ears position
        self.send_ears_state(emotions_arr)
        
        if self.action_executing:
            action = next(self.action_iter,None)
            
            if score < -0.25:
                # perform crossover and mutations
                self.action_executing = False
                self.stop()
                
                self.update_actions_for_embeddings(self.action_list,score)
                return
            
            if action is None or score > 0.0:
                # perform action set trim
                self.action_executing = False
                self.stop()
                
                self.update_actions_for_embeddings(self.action_list,score)
                return
            
            self.send_command(action[0],action[1])
            self.get_logger().info("Performing actions")
            
            return
        
        actions_lists = []
        
        # if score >= 0.0:
        #     self.stop()
        #     return
        
        # retrive actions associated with visual data        
        actions_lists.extend(
            self.image_action_retrival()
        )
        
        # retrive actions associated with audio data ( mel spectogram )
        
        
        # perform crossover and mutations on gathered actions
        
        sorted_actions_lists = sorted(actions_lists,
                                      key = lambda x: x['score'],
                                      reverse=True)
        
        self.action_list = self.action_crossovers(sorted_actions_lists)
        
        self.action_iter = iter(self.action_list)
        
        self.get_logger().info(f"Action list lenght: {len(self.action_list)}")
        
        self.action_executing = True
        
        
    def tick_callback(self):
        
        if self.wait_for_img or self.wait_for_depth:
            return
        
        self.get_logger().info('Mind tick')
        
        start = timer()
        
        self.tick_func()
        
        end = timer()
        
        self.get_logger().info(f'Tick inference time {end - start} s')
        

def main(args=None):
    rclpy.init(args=args)

    map_mind = MapMind()

    rclpy.spin(map_mind)

    map_mind.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()