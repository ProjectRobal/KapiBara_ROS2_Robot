#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Range,Imu,PointCloud2
from geometry_msgs.msg import Quaternion

from ament_index_python.packages import get_package_share_directory,get_package_prefix

from std_msgs.msg import Float64MultiArray

from kapibara.DeepIDTFLite import DeepIDTFLite

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

from ultralytics import YOLO

import webrtcvad

import cv2
from cv_bridge import CvBridge

import base64
import numpy as np

from timeit import default_timer as timer
import time

import librosa

from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance,OrderBy,PayloadSchemaType
from qdrant_client.http import models

import random

import uuid

from tiny_vectordb import VectorDatabase

ID_TO_EMOTION_NAME = [
    "angry",
    "fear",
    "happiness",
    "uncertainty",
    "boredom"
    ]

IMG_EMBEDDING_SHAPE = 32*32*4
ML_SPECTOGRAM_EMBEDDING_SHAPE = 32*32

# month, day, hour, minute, second
TIME_EMBEDDING_SHAPE = 5

IMG_ACTION_SET_NAME = "actions_sets"
SPECTOGRAM_ACTION_NAME = "ml_actions_sets"

GENERAL_POINT_DB_NAME = "general_point_database"
GENRAL_POINT_EMBEDDING_SHAPE = 64

GENERAL_ACTION_DB_NAME = "general_action_database"
GENRAL_POINT_EMBEDDING_SHAPE = 64

POINT_MAP_DB = "points_database"
POINT_IMG_DB = "points_imgages"
POINT_SPEC_DB = "points_spect"
POINT_TIME_DB = "points_time"


def split_embedding(input):
    
    x = 0
    y = 0
    
    steps = int(56/8)
    
    embeddings = []
    
    for y in range(steps):
        for x in range(steps):
            embeddings.append(
                input[x*8:x*8 + 8,y*8:y*8 + 8].flatten()
            )
        
    return embeddings
    

def generate_embedding(img,depth):
    '''
    generate_embedding 
    
    In that case it takes image and 
    depth image, merges them and split them
    into equal parts.
    
    :param img: Associated RGB image
    :param depth: Associated depth image
    '''
    
    img = cv2.resize(img,(56,56)).astype(np.float32) / 255.0
    b,g,r = cv2.split(img)
    
    img = 0.4*r + 0.5*g + 0.1*b
    
    depth = cv2.resize(depth,(56,56))
    
    img_merged = img+depth

    return split_embedding(img_merged)

def generate_embedding_spec(spec):
    
    return split_embedding(spec)


FACE_TOLERANCE = 0.94


class FaceSqlite:
    
    def __init__(self,db) -> None:
        
        self.db = db
        
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
        
        package_path = get_package_share_directory('kapibara')
        
        self.declare_parameter('yolo_model_path','yolov11n-face_float32.tflite')
        self.declare_parameter('deepid_model_path','deepid.tflite')
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        
        yolo_model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value
        
        self.face_yolo = YOLO(os.path.join(package_path,'model',yolo_model_path))
        
        deepid_model_path = self.get_parameter('deepid_model_path').get_parameter_value().string_value
        
        self.deep_id = DeepIDTFLite(filepath=os.path.join(package_path,'model',deepid_model_path))
        
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.emotion_pub = self.create_publisher(Emotions,'emotions', 10)
        
        self.spectogram_publisher = self.create_publisher(Image, 'spectogram', 10)
        
        self.timer_period = self.get_parameter('tick_time').get_parameter_value().double_value  # seconds
        self.timer = self.create_timer(self.timer_period, self.tick_callback)
        
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
        
        self.current_face_embeddings = []
        
        self.initialize_db()
        
        # self.face_db = FaceSqlite(self.db)
        
        self.pat_detected = 0.0
                
        self.emotion_state = 0.0
        
        self.last_img_embeddings = []
        
        self.last_spectogram_embeddings = []
        
        self.action_list = []
        self.action_executing = False
        self.action_iter = 0
        
        self.image = None
        self.depth = None
        self.spectogram = None
        
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
        
        self.last_score = 0
        
        # robot position in the map
        self.x = 0
        self.y = 0
        self.yaw = 0
        
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
        
    def reinitialize_db(self):
        
        self.initialize_db()
        
    def initialize_db(self):
        
        collection_configs = [
            {
                'name': GENERAL_POINT_DB_NAME,
                'dimension': GENRAL_POINT_EMBEDDING_SHAPE,
            },
            {
                'name': POINT_MAP_DB,
                'dimension': 2
            },
            {
                'name':'faces',
                'dimension':160
            }
        ]
        
        self.db = VectorDatabase("test.db", collection_configs)
        
        self.general_db = self.db[GENERAL_POINT_DB_NAME]
        self.point_db = self.db[POINT_MAP_DB]
        
            
    def get_points_in_radius(self,x:float,y:float,radius:float = 0.5)->list[tuple]:
        
        search_ids, search_scores = self.point_db.search([x,y], k=64)

        values = []
        points_id = []
        points = []
        for id,score in zip(search_ids, search_scores):
            if score > radius:
                continue
            
            payload = id.split(":")[1]
            values.append(float(payload))
            points_id.append(id)
            # points.append((,vec[1],float(payload)))

        for val,block in zip(values,self.point_db.getBlock(points_id)):
            points.append((block[0],block[1],val))
        
        return points

    def get_point_at_position(self,x:float,y:float,pop:bool = False)->float:
        """
        Docstring for get_point_at_position
        
        :param x: x position in 2D space
        :type x: float
        :param y: y position in 2D space
        :type y: float
        :param pop: whether to remove point from map
        :type pop: bool
        :return: associated value with point id in database
        :rtype: float
        """
        
        search_ids, search_scores = self.point_db.search([x,y], k=1)
        
        
        if len(search_ids) == 1 and \
            search_scores[0] < 0.01:

            value = float(search_ids[0].split(":")[1])
            
            if pop:
                # pop the point from the map
                self.point_db.deleteBlock(
                    [search_ids[0]]
                )
        
            return value

        return 0.0
    
    def push_point_at_position(self,x:float,y:float,value:float):
        
        search_ids, search_scores = self.point_db.search([x,y], k=1)

        if len(search_ids) == 1 and \
            search_scores[0] < 0.01:
            
            self.point_db.deleteBlock(
                [search_ids[0]]
            )

        new_id = str(uuid.uuid4())+":"+str(new_id)
        
        self.point_db.setBlock(
            [new_id],
            [[x,y]]
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
        
        # estimate position based on speed solely
        # it will be filter out using external odometry /
        # map info
        self.yaw += angular*self.timer_period
        
        self.x += linear*np.cos(self.yaw)*self.timer_period
        self.y += linear*np.sin(self.yaw)*self.timer_period
        
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
        
        spectogram = librosa.feature.melspectrogram(y=self.mic_buffor, sr=16000,n_mels=224,hop_length=143)
        
        self.get_logger().info(f"Spectogram size: {spectogram.shape}")
        
        # publish last spectogram
        self.spectogram_publisher.publish(self.bridge.cv2_to_imgmsg(spectogram))
                
        self.get_logger().debug("Hearing time: "+str(timer() - start)+" s")
        
        self.spectogram = cv2.resize(spectogram,(56,56))
        
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
        
        self.obstacle_detected = float(np.exp(-min_points*25))
        
        self.obstacle_detected = min(self.obstacle_detected,1.0)
        
        if self.obstacle_detected < 0.01:
            self.obstacle_detected = 0.0
                
        if self.obstacle_detected:    
            self.get_logger().info(f'Obstacle detected! {self.obstacle_detected}, min point {min_points}')
            
    def sense_callabck(self,sense:PiezoSense):
        
        # should be rewritten 
        
        pin_states = sense.pin_state
        
        patting_sense = pin_states[4]
                
        if patting_sense:
            
            self.get_logger().debug('Patting detected')
                        
            self.pat_detected = 1.0
                        
            score = 10
                                                    
            # self.stop_mind()
                        
            if score !=0 and len(self.current_face_embeddings)>0:
                # check if spotted face are present in database:
                    
                # sort by distances from robot                
                self.current_face_embeddings = sorted(self.current_face_embeddings,key=lambda x: x[1],reverse=True)
                
                nearest_face = self.current_face_embeddings[0]                
                
                self.face.update_face(nearest_face,score)
                
                            
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
                
    
    def add_point_to_embeddings(self,point:tuple,embeddings):
        """
        Docstring for add_point_to_embeddings
        
        :param self: Description
        :param point: list with points (x,y,v) - x,y position in 2D space
                v - associated value
        :type points: list[tuple]
        """
        
        points_id = []

        for embedding in embeddings:
            
            new_id = uuid.uuid4()
                                    
            n_points = np.array(point,dtype=np.float32)
            
            payload = n_points
            
            self.get_logger().debug("Updating image embedding.")

            payload = base64.b64encode(payload.tobytes())
            
            points_id.append(str(new_id)+":"+payload.decode('ascii'))
        
        self.general_db.setBlock(
            points_id,
            embeddings
        )
        
    def get_points_from_embeddings(self,embeddings)->list[tuple]:
        
        points = []
        
        requests = []
        
        for embedding in embeddings:
            
            # check if embeddings aren't aleardy presents
            
            search_ids, search_scores = self.general_db.search(embedding, k=4)
            
            for id,score in zip(search_ids, search_scores):
                if score > 0.1:
                    continue
                    
                payload = id.split(":")[1].encode('ascii')
                payload = base64.b64decode(payload)
                
                point = np.frombuffer(payload, dtype=np.float32)
                
                points.append(point)
        
        return points
    
                
    def tick_func(self):
        # emotion validation pipeline
        
        # face detection
        
        # faces = self.face_yolo(self.image)
        
        # mean_embed = np.zeros(160,dtype=np.float32)
        
        # sum_distances = 0
        
        # Maybe we should take into account only the closest
        # face ?
        
        self.current_face_embeddings.clear()
        
        # examine resulted faces
        # for result in faces:
        #     boxes = result.boxes
        #     for box in boxes:
        #         x1, y1, x2, y2 = map(int, box.xyxy[0])
                
        #         width = x2 - x1
        #         height = y2 - y1
                
        #         distance = width*height
                
        #         conf = box.conf[0]
        #         cls = int(box.cls[0])
                
        #         img = self.image[y1:y2,x1:x2]
                
        #         embed = np.array(self.deep_id.process(img)[0],dtype=np.float32)

        #         self.current_face_embeddings.append([embed,distance])
        
        # evaluate emotion states
        
        emotions = Emotions()
        
        self.emotion.happiness = self.pat_detected*10.0
        self.emotion.fear = self.obstacle_detected*1.0
        
        self.pat_detected = 0.0
        
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
        
        dscore = score - self.last_score

        self.last_score = score
        
        # send ears position
        self.send_ears_state(emotions_arr)
        
        # It looks like 224x224 splitted into 49 parts of 32x32 is
        # a bit too much for Qdrant to handle in real time
        # 
        # we can reduce the resolution of input image and thus
        # number of embeddings generated or reduce the 
        # dimensionality of embeddings ...
        #
        # Apparently reducing resolution to 56x56 and splitting
        # into 49 parts of 8x8 gives medicore results but 
        # for single 8x8 embedding we can retrieve points in real time
        # but is the 8x8 too small?
        
        
        if score < 0:
            # if image and depth is present
            if self.image is not None and self.depth is not None:
                # generate embeddings
                self.last_img_embeddings = generate_embedding(self.image,self.depth)
                # gather points associated with embeddings
                points = self.get_points_from_embeddings(self.last_img_embeddings)
        
                # # update emotion map with those points
                for p in points:
                    x = p[0]
                    y = p[1]
                    v = p[2]
                    
                    self.push_point_at_position(x,y,v)
                                
                # self.get_logger().info(f"Retrieved {len(points)} points from map")
                # update image embeddings with current points in map
                
                # a major slow down
                self.add_point_to_embeddings((self.x,self.y,score),self.last_img_embeddings)

            # update current position point
            for point in points:
                self.push_point_at_position(point[0],point[1],score)
        
        return
        
        if self.action_executing:
            
            action = self.action_list[self.action_iter]
            
            self.action_iter += 1
            
            if dscore < -0.25:
                # perform mutations
                self.action_executing = False
                self.stop()
                
                self.mutate_actions(self.action_list,threshold=0.25,mean=0.5)
                
                # self.update_actions_for_embeddings(self.action_list,score)
                return
            
            if self.action_iter == len(self.action_list) or score > 0.0:
                # perform action set trim
                self.action_list = self.action_list[:self.action_iter]
                
                self.action_executing = False
                self.stop()
                
                # self.update_actions_for_embeddings(self.action_list,score)
                return
            
            self.send_command(action[0],action[1])
            self.get_logger().info("Performing actions")
            
            return
        
        actions_lists = []
        
        # when our robot is calm there is no need to take any actions ...
        if score >= 0.0:
            self.stop()
            return
        
        # retrive actions associated with visual data        
        # actions_lists.extend(
        #     self.image_action_retrival()
        # )
        
        # retrive actions associated with audio data ( mel spectogram )
        
        # actions_lists.extend(
        #     self.spectogram_action_retrival()
        # )        
        
        # perform crossover and mutations on gathered actions
        
        # sorted_actions_lists = sorted(actions_lists,
        #                               key = lambda x: x['score'],
        #                               reverse=True)
        
        # if len(sorted_actions_lists) > 0 and sorted_actions_lists[0]['score'] > 0.0:
        #     self.action_list = sorted_actions_lists[0]["actions"]
        # else:
        #     self.action_list = self.action_crossovers(sorted_actions_lists)
        
        self.action_iter = 0
        
        self.get_logger().info(f"Action list lenght: {len(self.action_list)}")
        
        self.action_executing = True
        
        
    def tick_callback(self):
        
        if self.wait_for_img or self.wait_for_depth or self.spectogram is None:
            return
        
        self.timer.cancel()
        
        self.get_logger().info('Mind tick')
        
        start = timer()
        
        self.tick_func()
        
        end = timer()
        
        self.get_logger().info(f'Tick inference time {end - start} s')
        
        self.timer.reset()
        

def main(args=None):
    rclpy.init(args=args)

    map_mind = MapMind()

    rclpy.spin(map_mind)

    map_mind.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()