#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from rclpy.executors import MultiThreadedExecutor

from scipy.signal import spectrogram


from sensor_msgs.msg import Range,Imu,PointCloud2
from geometry_msgs.msg import Quaternion

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

from threading import Lock


import signal

import os
import json

import cv2
from cv_bridge import CvBridge

from nav_msgs.msg import Odometry

from timeit import default_timer as timer
import time

from copy import copy
import numpy as np

from ament_index_python.packages import get_package_share_directory,get_package_prefix


from emotion_estimer.DeepIDTFLite import DeepIDTFLite
from emotion_estimer.TFLiteFaceDetector import UltraLightFaceDetecion

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from emotion_estimer.face_data import FaceData,FaceObj

import webrtcvad

import sqlite3
import sqlite_vec


FACE_TOLERANCE = 0.94


class FaceSqlite:
    
    def __init__(self) -> None:
        
        self.db = sqlite3.connect("faces.db")
        self.db.enable_load_extension(True)
        sqlite_vec.load(self.db)
        self.db.enable_load_extension(False)
        
        self.init_databse()
        
    def init_databse(self):
        
        self.db.execute(
            """
                CREATE VIRTUAL TABLE IF NOT EXISTS vec_faces USING vec0(
                score FLOAT,
                time INT64,
                face_embedding FLOAT[160]
                );
            """
        )
    
    def get_face(self,face_embedding:np.ndarray):
        rows = self.db.execute(
                """
                SELECT
                    rowid,
                    score,
                    distance
                FROM vec_faces
                WHERE face_embedding MATCH ?
                ORDER BY distance
                LIMIT 3
                """,
                [face_embedding.tobytes()],
            ).fetchall()
        
        for row in rows:
            if row[2] < 0.1:
                return row
            
    def check_size(self):
        
        rows = self.db.execute(
            """
            SELECT COUNT(*) FROM vec_faces;
            """
        ).fetchall()
        
        return rows[0][0]
        
    def update_face(self,face_embedding:np.ndarray,score:float):
        
        # check size
        
        size = self.check_size()
        
        if size >= 500:
            rows = self.db.execute(
                """
                SELECT rowid,time FROM vec_faces ORDER BY time LIMIT 1;
                """
            ).fetchall()
            
            id = rows[0][0]
            
            with self.db:
                self.db.execute(
                    """
                    UPDATE vec_faces SET score = ?, face_embedding = ? WHERE rowid = ?;
                    """,
                    [score,face_embedding.tobytes(),id]
                )
            
        
        # check if face exists
        
        face = self.get_face(face_embedding)
        
        if face is None:
            with self.db:
                self.db.execute(
                    """
                    INSERT INTO vec_faces(score, time, face_embedding) VALUES(?, ?, ?);      
                    """,
                    [score,time.time_ns(),face_embedding.tobytes()]
                    )
        else:
            
            id = face[0]
            
            with self.db:
                self.db.execute(
                    """
                    UPDATE vec_faces SET score = ? WHERE rowid = ?;
                    """,
                    [score,id]
                )
            
            
    def __del__(self):
        self.db.commit()


class EmotionEstimator(Node):

    def __init__(self):
        super().__init__('emotion_estimator')
        
        self.current_ranges=[]
        
        self.id_to_emotion_name = ["angry","fear","happiness","uncertainty","boredom"]
        
        self.declare_parameter('camera_topic', 'camera/image_raw/compressed',descriptor=ParameterDescriptor(name='camera_topic', type=0, description="A topic to subscribe for compressed image"))
        
        self.declare_parameter('range_threshold', 0.1)
        self.declare_parameter('accel_threshold', 10.0)
        self.declare_parameter('angular_threshold', 1.0)
        
        # models paths
        # self.declare_parameter('midas_model', 'Midas-V2-Quantized_edgetpu.tflite')
        # self.declare_parameter('deepid_model', 'deepid_edgetpu.tflite')
        # self.declare_parameter('face_model', 'slim_edgetpu.tflite')
        # self.declare_parameter('audio_model', 'model_edgetpu.tflite')
        
        self.declare_parameter('sim',False)
        
        # a topic to send ears position
        
        self.declare_parameter('ears_topic','ears_controller/commands')
        
        # a orientation callback
        self.declare_parameter('imu', 'imu')
        
        # a topic to listen for audio from microphone
        self.declare_parameter('mic', 'mic')
        
        # a topic to listen for odometry 
        self.declare_parameter('odom','motors/odom')
        
        # piezo sense sensors
        self.declare_parameter('sense_topic', 'sense')
        
        # face database file
        self.declare_parameter('face_db', '/app/src/face')
        
        sim:bool = self.get_parameter('sim').get_parameter_value().bool_value
        
        deepid_model:str = 'deepid_edgetpu.tflite' if not sim else 'deepid.tflite'
        face_model:str = 'slim_edgetpu.tflite' if not sim else 'slim.tflite'
        
        # VAD init
        
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)
        
        # store 500 face embeddings + rewards
        self.embeddings_buffor = np.zeros((500,160),dtype=np.float32)
        
        
        self._emotions_lock = Lock()
        self._face_lock = Lock()
        
        
        self.current_embeddings = []
        # angry
        # fear
        # happiness
        # uncertainty
        # boredom 
        # anguler values for each emotions state
        self.emotions_angle=[0.0,180.0,25.0,145.0,90.0]  
        
        # Sensor callback group
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Callback group for camera and faces
        self.camera_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Callback group for mic
        self.mic_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Callback group for others timers
        self.timers_callback_group = MutuallyExclusiveCallbackGroup()
          
        
        self.ears_publisher = self.create_publisher(Float64MultiArray, self.get_parameter('ears_topic').get_parameter_value().string_value, 10)
       
        self.emotion_publisher = self.create_publisher(Emotions,"emotions",10)
        
        
        self.bridge = CvBridge()
        
        model_path = os.path.join(get_package_share_directory('emotion_estimer'),'model',face_model)
        
        self.get_logger().info('Initializing LiteFaceDetector...')
        
        self.face_detect = UltraLightFaceDetecion(filepath=model_path)
        
        self.get_logger().info('Model initialized!')
        
        
        model_path = os.path.join(get_package_share_directory('emotion_estimer'),'model',deepid_model)
        
        self.get_logger().info('Initializing LiteFaceDetector...')
        
        self.deep_id = DeepIDTFLite(filepath=model_path)
        
        self.get_logger().info('Model initialized!')
                
        self.spectogram_publisher = self.create_publisher(Image, 'spectogram', 10)
        
        self.subscription = self.create_subscription(
            CompressedImage,
            self.get_parameter('camera_topic').get_parameter_value().string_value,
            self.camera_listener,
            10)
        self.subscription  # prevent unused variable warning
        
        
        self.accel_threshold = self.get_parameter('accel_threshold').get_parameter_value().double_value
        self.last_accel = 0.0
        self.thrust = 0.0
        self.last_uncertanity = 0.0
        
        self.thrust_fear = 0.0
        
        self.angular_threshold = self.get_parameter('angular_threshold').get_parameter_value().double_value
        self.last_angular = 0.0
        self.angular_jerk = 0.0
        self.last_angular_fear = 0.0
        
        self.jerk_fear = 0.0
        
        self.procratination_counter = 0
        self.procratination_timer = self.create_timer(2.0, self.procratination_timer_callback)
        
        # parameter that describe fear distance threshold for laser sensor
        
        
        self.range_threshold = self.get_parameter('range_threshold').get_parameter_value().double_value
        
        # Orienataion callback
        
        imu_topic = self.get_parameter('imu').get_parameter_value().string_value
        
        self.get_logger().info("Creating subscription for IMU sensor at topic: "+imu_topic)
        self.imu_subscripe = self.create_subscription(Imu,imu_topic,self.imu_callback,10)
        
        # Sense callback
        
        sense_topic = self.get_parameter('sense_topic').get_parameter_value().string_value
        
        self.get_logger().info("Creating subscription for Pizeo Sense at topic: "+sense_topic)
        self.sense_subscripe = self.create_subscription(PiezoSense,sense_topic,self.sense_callabck,10)
    
        
        # Microphone callback use audio model 
        
        mic_topic = self.get_parameter('mic').get_parameter_value().string_value
        
        self.mic_buffor = np.zeros(2*16000,dtype=np.float32)
        
        self.get_logger().info("Creating subscription for Microphone at topic: "+mic_topic)
        self.mic_subscripe = self.create_subscription(Microphone,mic_topic,self.mic_callback,1)
        
        # Subcription for odometry
        
        odom_topic = self.get_parameter('odom').get_parameter_value().string_value
        self.get_logger().info("Creating subscription for odometry sensor at topic: "+odom_topic)
        self.odom_subscripe = self.create_subscription(Odometry,odom_topic,self.odom_callback,10)
        
        self.get_logger().info("Creating publisher for spoted faces")
        self.face_publisher = self.create_publisher(FaceEmbed, 'spoted_faces', 10)
                
        self.move_lock = False
        
        self.pain_value = 0
        
        self.good_sense = 0
        
        self.uncertain_sense = 1.0
        
        self.uncertain_speech = 0.0
        
        self.points_covarage = 0
        
        self._last_sense = 0
        self._thrust = 0
        self._patting_counter = 0
        self.last_delta = 0.0
        
        
        self.found_wall = False
        # it is not needed
        
        # points_topic = self.get_parameter('points_topic').get_parameter_value().string_value
        
        # self.get_logger().info("Creating subscription for Point Cloud sensor at topic: "+points_topic)
        # self.point_subscripe = self.create_subscription(PointCloud2,points_topic,self.points_callback,10)
                
        self.ears_timer = self.create_timer(0.05, self.ears_subscriber_timer)
        
        self.face_commit_timer = self.create_timer(60*30, self.commit_faces)
        
        # each 30 minutes
        self.save_face_timer = self.create_timer(30*60, self.save_face_timer_callback)
        
        self.face_db_name:str = self.get_parameter('face_db').get_parameter_value().string_value+".db"
        
        self.face_score_name = self.face_db_name+"_scores.json"
        
        self.audio_output = 0
        
        self.audio_fear = 0
        
        
        # self.face_database = VectorDatabase(self.face_db_name,[{
        # "name":"faces",
        # "dimension":160
        # }])
        
        self.faces = FaceSqlite()
        
        # self.faces = self.face_database['faces']
                
        # self.faces_score = FaceData(self.face_score_name)
        
        self.skip_frames_counter = 0
        
        # KapiBara mind stop timer:
        
        self.start_again_mind = self.create_timer(15,self.start_again_mind_callback)
        self.start_again_mind.cancel()
        
        # self.stop_mind_srv = self.create_client(StopMind,'stop_mind')
        
        timeout_counter = 0
        
        self._stop_mind_failed = False
                
        # while not self.stop_mind_srv.wait_for_service(timeout_sec=20.0):
        #     self.get_logger().info('Stop Mind service not available, waiting again...')
            
        #     timeout_counter += 1
        #     if timeout_counter > 60:
        #         self.get_logger().error('Stop Mind service not available, exiting...')
                
        #         self._stop_mind_failed = True
                
        #         break
            
        # if not self.stop_mind_srv.service_is_ready():
        #     self.get_logger().error("Stop Mind service is not available!!!")
    
    
    def stop_mind(self):
        
        if self._stop_mind_failed:
            return
        
        self.get_logger().info('Stopping Mind ...')
        
        req = StopMind.Request()
        
        req.stop = True
        
        future = self.stop_mind_srv.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info('Mind stopped!')
        
        self.start_again_mind.reset()
        
        
    
    def start_again_mind_callback(self):
        
        if self._stop_mind_failed:
            return
        
        self.get_logger().info('Starting Mind ...')
        
        req = StopMind.Request()
        
        req.stop = False
        
        future = self.stop_mind_srv.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        
        self.start_again_mind.cancel()
        
        self.get_logger().info('Mind started!')
    
        
    def commit_faces(self,timeout=False):
        
        self._face_lock.acquire( timeout = 60 if timeout else 0 )
        
        self.get_logger().info("Saving faces data!")
        
        # self.face_database.commit()
        # self.faces_score.commit()
        
        self._face_lock.release()
        
    def search_ids_to_num(self,ids:str)->int:
        return int(ids[3:])
        
    def save_face_timer_callback(self):
        self.get_logger().debug("Saving face callback!")
        self.save_faces()
    
    # subscribe ears position based on current emotion 
    def ears_subscriber_timer(self):
        emotions = self.calculate_emotion_status()
        
        _emotions = Emotions()
        
        _emotions.angry = emotions[0]
        _emotions.fear = emotions[1]
        _emotions.happiness = emotions[2]
        _emotions.uncertainty = emotions[3]
        _emotions.boredom = emotions[4]
        
        self.emotion_publisher.publish(_emotions)
        
        max_id = 4
        
        if np.sum(emotions) >= 0.01:
            max_id:int = np.argmax(emotions[:4])
            
        self.get_logger().debug("Current emotion: "+str(self.id_to_emotion_name[max_id]))
            
        self.get_logger().debug("Sending angle to ears: "+str(self.emotions_angle[max_id]))
        
        angle:float = (self.emotions_angle[max_id]/180.0)*np.pi
        
        array=Float64MultiArray()
        
        array.data=[np.pi - angle, angle]
        
        self.ears_publisher.publish(array)
    
    
    def calculate_emotion_status(self) -> np.ndarray:
        emotions=np.zeros(5)
        
        face_score = 0
        
        unknow_face = 0
        
        self._face_lock.acquire()
        
        if len(self.current_embeddings)>0:
            # sort by distances from robot                
            self.current_embeddings = sorted(self.current_embeddings,key=lambda x: x[1],reverse=True)
            
            nearest_face = self.current_embeddings[0]
            
            face = self.faces.get_face(nearest_face)
            
            # search_ids, search_scores = self.faces.search(nearest_face[0],k=10)
            
            if face is not None:
                                                                         
                face_score = face[1]
                                                    
                self.faces.update_face(nearest_face,face_score)
                
                self.get_logger().info('Face with id '+face[0]+' change emotion state with score '+str(face_score))
                
            else:
                unknow_face = 1
                
                    
        
        self._face_lock.release()
        
        
        self._emotions_lock.acquire()
        
        if self.pain_value > 0.5:
            self.good_sense = 0.0
                    
        # anger
        # fear
        # happiness
        # uncertainty
        # bordorm
        
        # I need to adjust sound model so anger will be zero for now
        emotions[0] = (self.audio_output == 4)*0.1
        emotions[1] = (self.audio_output == 3)*0.1 + self.audio_fear*0.25 + self.thrust_fear*0.25 + ( face_score < 0 )*0.25  + 4.0*self.pain_value
        emotions[2] = (self.audio_output == 2)*0.1 + self.good_sense + ( face_score > 0 )*0.5
        emotions[3] = (self.audio_output == 1)*0.1 + self.jerk_fear*0.25 + self.found_wall*0.65 + self.uncertain_sense*0.5 + unknow_face*0.1 + self.uncertain_speech*0.1
        emotions[4] = np.floor(self.procratination_counter/10.0)
        
        self.pain_value = self.pain_value / 1.25
        self.good_sense = self.good_sense / 1.5
        self.uncertain_sense = self.uncertain_sense / 1.35
        self.thrust_fear = self.thrust_fear / 2
        self.jerk_fear = self.jerk_fear / 2
        self.audio_fear = self.audio_fear / 1.5
        
        self.uncertain_sense = 0.0
        
        if self.audio_fear <= 0.01:
            self.audio_fear = 0.0
        
        if self.pain_value <= 0.01:
            self.pain_value = 0.0
            
        if self.good_sense <= 0.01:
            self.good_sense = 0.0
            
        if self.uncertain_sense <= 0.01:
            self.uncertain_sense = 0.0
            
        if self.thrust_fear <= 0.01:
            self.thrust_fear = 0.0
            
        if self.jerk_fear <= 0.01:
            self.jerk_fear = 0.0
            
        self._emotions_lock.release()
        
        return emotions
        
           
    def procratination_timer_callback(self):
        
        self._emotions_lock.acquire()
        
        if self.procratination_counter < 10000:
            self.procratination_counter = self.procratination_counter + 1
            
        self._emotions_lock.release()
            
    def camera_listener(self, msg:CompressedImage):
        self.get_logger().debug('I got image with format: %s' % msg.format)
        
        self.skip_frames_counter+=1
        
        if self.skip_frames_counter < 3:
            return
        
        self.skip_frames_counter = 0
        
        image = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        # face detection:
        
        self._face_lock.acquire()
        
        # since I don't have appropiate model we left it empty for now
        
        boxes,scores = self.face_detect.inference(image)
        
        self.current_embeddings.clear()
        
        mean_embed = np.zeros(160,dtype=np.float32)
        
        sum_distances = 0
        
        # A mean embedding takes into account distance between robot and face.
        
        for box in boxes.astype(int):
            
            img = image[box[1]:box[3],box[0]:box[2]]
            
            width = box[3] - box[1]
            height = box[2] - box[0]
            
            distance = width*height
            
            sum_distances += distance
            
            embed = np.array(self.deep_id.process(img)[0],dtype=np.float32)
            
            mean_embed += embed*distance
            
            # set of embedding and estimated distance
            self.current_embeddings.append([embed,distance])
        
        if len(boxes) != 0:
            mean_embed /= sum_distances
        
        face = FaceEmbed()
        
        face.embedding = mean_embed.tolist()
        
        self.face_publisher.publish(face)
        
        self._face_lock.release()
        
        # finding wall routine , we search for two white dots caused by IR lights from camera
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        ret,th1 = cv2.threshold(gray,200, 255,cv2.THRESH_BINARY)
        
        contours, _ = cv2.findContours(th1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        found_rects:int = 0
        
        for i, contour in enumerate(contours):
            if i == 0:
                continue
            
            x,y,w,h = cv2.boundingRect(contour)
            
            if w > 45 and h > 45:
                found_rects += 1
        
        self._emotions_lock.acquire()
        
        if found_rects == 2:
            self.found_wall = True
            # self.get_logger().info('Found wall!!')
        else:
            self.found_wall = False
        
        self._emotions_lock.release()
        
            
    def sense_callabck(self,sense:PiezoSense):
        
        # should be rewritten 
        
        pin_states = sense.pin_state
        
        front_bumper = pin_states[2]
        back_bumper = pin_states[3]
        
        patting_sense = pin_states[4]
        
        if not front_bumper or not back_bumper:
            self.get_logger().info("Pain occured by bumper!")
            
            self._emotions_lock.acquire()
            
            self.pain_value = 10.0
            
            self._emotions_lock.release()
                
        if patting_sense:
            
            self.get_logger().debug('Patting detected')
            
            
            self._emotions_lock.acquire()
            
            self.good_sense = 10.0
            
            self._emotions_lock.release()
            
            
            score = 10
                                                    
            # self.stop_mind()
            
            self._face_lock.acquire()
            
            if score !=0 and len(self.current_embeddings)>0:
                # check if spotted face are present in database:
                    
                # sort by distances from robot                
                self.current_embeddings = sorted(self.current_embeddings,key=lambda x: x[1],reverse=True)
                
                nearest_face = self.current_embeddings[0]                
                
                out = self.face.update_face(nearest_face,score)
                
                if out == -1:
                    self.get_logger().info("Face with has been added!")
                elif out > 0:
                    self.get_logger().info(f"Face with id {out} has been updated!")
                else:
                    self.get_logger().info("Face with has been overrided!")
                
            self._face_lock.release()
        
    
    def odom_callback(self,odom:Odometry):
        
        velocity = odom.twist.twist.linear.x*odom.twist.twist.linear.x + odom.twist.twist.linear.y*odom.twist.twist.linear.y + odom.twist.twist.linear.z*odom.twist.twist.linear.z
        angular = odom.twist.twist.angular.x*odom.twist.twist.angular.x + odom.twist.twist.angular.y*odom.twist.twist.angular.y + odom.twist.twist.angular.z*odom.twist.twist.angular.z
        
        self.get_logger().debug("Odom recive velocity of "+str(velocity)+" and angular velocity of "+str(angular))
        if velocity > 0.0001 or angular > 0.00001:
            
            self._emotions_lock.acquire()
            
            self.move_lock = True
            self.procratination_counter = 0
            
            self._emotions_lock.release()
            
        else:
            
            self._emotions_lock.acquire()
            
            self.move_lock = False       
            
            self._emotions_lock.release() 
        
    def imu_callback(self,imu:Imu):
        accel=imu.linear_acceleration
        
        accel_value = abs(np.sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z) - 9.81)
                
        if accel_value > 10.0:
            self.get_logger().info("Pain occured! (imu)")
            
            self._emotions_lock.acquire()
            
            self.pain_value = 1.0
            
            self._emotions_lock.release()
            
            score = -10.0
            
            self._face_lock.acquire()
            
            if len(self.current_embeddings)>0:
                # check if spotted face are present in database:
                    
                # sort by distances from robot                
                self.current_embeddings = sorted(self.current_embeddings,key=lambda x: x[1],reverse=True)
                
                nearest_face = self.current_embeddings[0]                
                
                out = self.face.update_face(nearest_face,score)
                
                if out == -1:
                    self.get_logger().info("Face with has been added!")
                elif out > 0:
                    self.get_logger().info(f"Face with id {out} has been updated!")
                else:
                    self.get_logger().info("Face with has been overrided!")
            
            
            self._face_lock.release()
        
        self.thrust = abs(accel_value-self.last_accel)
        
        self.last_accel = accel_value
        
        if self.move_lock:
            self.thrust = 0

        if self.thrust > 3.0:
            self.get_logger().info("Too harsh force")
            
            self._emotions_lock.acquire()
            
            self.thrust_fear = 1.0
            
            self.procratination_counter = 0
            
            self._emotions_lock.release()
        
        angular = imu.angular_velocity
        
        angular_value = np.sqrt(angular.x*angular.x + angular.y*angular.y + angular.z*angular.z)/3
        
        self.angular_jerk = abs(angular_value-self.last_angular)
        
        self.last_angular = angular_value
        
        if self.move_lock:
            self.angular_jerk = 0
        
        if self.angular_jerk > 40.0:        
            self.get_logger().info("I feel dizzy")
            
            self._emotions_lock.acquire()
            
            self.jerk_fear = 1.0
            
            self.procratination_counter = 0
            
            self._emotions_lock.release()
    
    def mic_callback(self,mic:Microphone):
        # I have to think about it
        
        self.mic_buffor = np.roll(self.mic_buffor,mic.buffor_size)
        
        left = np.array(mic.channel1,dtype=np.float32)/np.iinfo(np.int32).max
        right = np.array(mic.channel2,dtype=np.float32)/np.iinfo(np.int32).max
        
        combine = ( left + right ) / 2.0
        
        self.mic_buffor[:mic.buffor_size] = combine[:]
        
        start = timer()
        
        speech_detect = self.vad.is_speech(self.mic_buffor, 16000)
        
        if speech_detect:
            self.uncertain_speech = 1.0
        # We are going to replace it with something simpler
        # output,spectogram = self.hearing.input(self.mic_buffor)
        
        nperseg = 255
        noverlap = nperseg - 128 # 255 - 128 = 127

        # Calculate the spectrogram
        # f: array of sample frequencies
        # t_spec: array of segment times
        # Sxx: Spectrogram of x. By default, the last axis of Sxx corresponds to the segment times.
        f, t_spec, _spectogram = spectrogram(self.mic_buffor, fs=16000, nperseg=nperseg, noverlap=noverlap)
                
        _spectogram = cv2.normalize(_spectogram,None,alpha=0,beta=255,norm_type=cv2.NORM_MINMAX).astype(np.uint8)
        
        # publish last spectogram
        self.spectogram_publisher.publish(self.bridge.cv2_to_imgmsg(_spectogram))
                
        self.get_logger().debug("Hearing time: "+str(timer() - start)+" s")
        
        # self.get_logger().debug("Hearing output: "+str(self.hearing.answers[output]))
        
        self._emotions_lock.acquire()
        
        # self.audio_output = output
        
        mean = np.mean(np.abs(combine))
        
        if mean >= 0.7:
            self.audio_fear = 1.0
        
        self._emotions_lock.release()
        
    def save_faces(self):
        
        self._face_lock.acquire()
        
        self.get_logger().info("Saving faces embeddings!")
        
        # self.face_database.commit() 
        
        self._face_lock.release()     
        
    def on_shutdown(self):
        self.commit_faces( timeout = True )


def main(args=None):
    rclpy.init(args=args)

    emotion_estimator = EmotionEstimator()
    
    def on_sigint(sig,frame):
        
        rclpy.shutdown()
        
        emotion_estimator.on_shutdown()
        
        print("Shutdown")
        exit(0)
        
    
    signal.signal(signal.SIGINT,on_sigint)
    
    rclpy.spin(emotion_estimator)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()