#!/usr/bin/env python3

# Author: Eric Eaton, University of Pennsylvania
# Date: 2024-09-07
#
# ROS node for interfacing with the Acconeer XE125 board
# Code partially based on Acconeer's breathing detection example app:
# https://github.com/acconeer/acconeer-python-exploration/blob/master/examples/a121/algo/breathing/breathing_with_gui.py

import rospy
import numpy as np
from std_msgs.msg import Float32, String, UInt8
import acconeer.exptool as et
from acconeer.exptool import a121
from acconeer.exptool.a121.algo.breathing import AppState, RefApp
from acconeer.exptool.a121.algo.breathing._ref_app import BreathingProcessorConfig, RefAppConfig, get_sensor_config
from acconeer.exptool.a121.algo.presence import ProcessorConfig as PresenceProcessorConfig
from acconeer.exptool.a121.algo._utils import get_distances_m

from acconeer_radar_sensor.msg import Presence, Respiration

class PresenceData:
    def __init__(self):
        self.inter = None
        self.intra = None
        self.distances_being_analyzed = None
        self.status = ""
    def get_ROS_message(self):
        message = Presence()
        message.inter = self.inter.tolist() if isinstance(self.inter, np.ndarray) else self.inter  # Convert numpy array to list
        message.intra = self.intra.tolist() if isinstance(self.intra, np.ndarray) else self.intra  # Convert numpy array to list
        # Ensure distances_being_analyzed is either a list/tuple or set to an empty list
        if isinstance(self.distances_being_analyzed, np.ndarray):
            message.distances_being_analyzed = self.distances_being_analyzed.tolist()
        elif isinstance(self.distances_being_analyzed, list) or isinstance(self.distances_being_analyzed, tuple):
            message.distances_being_analyzed = self.distances_being_analyzed
        else:
            message.distances_being_analyzed = []  # Default to an empty list if None or invalid
        return message


class BreathingData:
    def __init__(self):
        self.breathing_rate = None
        self.time_vector = None
        self.motion = None
        self.psd = None
        self.psd_frequencies = None
        self.all_breathing_rate_history = None
        self.breathing_rate_history = None
        self.status = ""
    def get_ROS_message(self):
        message = Respiration()
        message.respiration_rate = self.breathing_rate
        message.time_vector = self.time_vector.tolist() if isinstance(self.time_vector, np.ndarray) else self.time_vector
        message.motion = self.motion.tolist() if isinstance(self.motion, np.ndarray) else self.motion
        message.psd = self.psd.tolist() if isinstance(self.psd, np.ndarray) else self.psd
        message.psd_frequencies = self.psd_frequencies.tolist() if isinstance(self.psd_frequencies, np.ndarray) else self.psd_frequencies
        message.all_respiration_rate_history = (
            self.all_breathing_rate_history.tolist() if isinstance(self.all_breathing_rate_history, np.ndarray) else self.all_breathing_rate_history)
        message.respiration_rate_history = (
            self.breathing_rate_history.tolist() if isinstance(self.breathing_rate_history, np.ndarray) else self.breathing_rate_history)
        return message

class TriggerHandler:

    def __init__(self):
        self.val_ = 0
        self.time_ = 0
        self.sent_ = False

    def __call__(self):
        return self.val_
    
    @property
    def time(self) -> float:
        now = rospy.get_time() 
        return now - self.time_

    @property
    def sent(self) -> bool:
        return self.sent_

    def reset(self):
        self.sent_ = True
        self.time_ = 0

    def callback(self, msg : UInt8) -> None:
        self.val_ = msg.data        
        if self.val_ != 0:
            self.sent_ = False
            self.time_ = rospy.get_time()
        
def sensor_data_publisher():
    rospy.init_node('respiration_radar_sensor', anonymous=True)

    args = a121.ExampleArgumentParser().parse_args()
    et.utils.config_logging(args)

    trigger = TriggerHandler()

    # ROS configuration
    frequency_sensor_probing = 120 # Sensor data sampling rate in Hz; I think the sensor data comes in around 44 Hz
    frequency_ros_message = 4      # Frequency to output sensor data to ROS in Hz. Must be <= frequency_sensor_probing
    frequency_ros_log = 1          # Frequency to output log to ROS in Hz. Must be <= frequency_ros_message
    
    # Sensor and configuration setup
    sensor = 1
    breathing_processor_config = BreathingProcessorConfig(
        lowest_breathing_rate=6,		
        highest_breathing_rate=60,		
        time_series_length_s=20,
    )
    presence_config = PresenceProcessorConfig(
        intra_detection_threshold=4,		
        intra_frame_time_const=0.15,		
        inter_frame_fast_cutoff=20,		
        inter_frame_slow_cutoff=0.2,		
        inter_frame_deviation_time_const=0.5,	
    )
    ref_app_config = RefAppConfig(
        use_presence_processor=True,
        num_distances_to_analyze=3,  		# num distances to analyze, centered around the range
        start_m=.8,  				# start of measurement range in meters
        end_m=1.5,   				# end of measurement range in meters
        distance_determination_duration=3,  	# how long in seconds to determine presence
        frame_rate=20,  			# default = 20
        sweeps_per_frame=16, 			# default = 16
        breathing_config=breathing_processor_config,
        presence_config=presence_config,
    )

    # Prepare client and sensor
    sensor_config = get_sensor_config(ref_app_config=ref_app_config)
    client = a121.Client.open(**a121.get_client_args(args))
    metadata = client.setup_session(sensor_config)
    
    distances = get_distances_m(sensor_config, metadata)            

    ref_app = RefApp(client=client, sensor_id=sensor, ref_app_config=ref_app_config)
    ref_app.start()

    rospy.Subscriber("/jackal_teleop/trigger", UInt8, trigger.callback)

    # Publishers for different data streams
    presence_pub = rospy.Publisher('/acconeer/presence', Presence, queue_size=10)
    breathing_rate_pub = rospy.Publisher('/acconeer/respiration_rate', Float32, queue_size=10)
    breathing_pub = rospy.Publisher('/acconeer/respiration', Respiration, queue_size=10)
    presence_status_pub = rospy.Publisher('/acconeer/presence_status', String, queue_size=10)
    breathing_status_pub = rospy.Publisher('/acconeer/respiration_status', String, queue_size=10)

    rate_db = [0]
    
    rate = rospy.Rate(frequency_sensor_probing)
    sensor_probing_counter = 0 

    # Loop to read sensor data
    while not rospy.is_shutdown():
        ref_app_result = ref_app.get_next()
        sensor_probing_counter = (sensor_probing_counter + 1) % frequency_sensor_probing
        #print(str(sensor_probing_counter)+ " " + str(((sensor_probing_counter * frequency_ros_log) % frequency_sensor_probing)==0))

        if ref_app_result is None:
            rospy.logwarn("Acconeer: No valid sensor data received.")
            continue
            
        #if ((sensor_probing_counter * frequency_ros_message) % frequency_sensor_probing) == 0:
        #    continue
        if trigger() > 0:
            try:
                
                presence = PresenceData()
                breathing = BreathingData()

                app_state = ref_app_result.app_state

                # Presence results (inter, intra, distances, status)
                presence.inter = ref_app_result.presence_result.inter
                presence.intra = ref_app_result.presence_result.intra
                presence.distances_being_analyzed = ref_app_result.distances_being_analyzed
                
                # Presence detection status
                if app_state == AppState.NO_PRESENCE_DETECTED:
                    presence.status = "No presence detected"
                elif app_state == AppState.DETERMINE_DISTANCE_ESTIMATE:
                    presence.status = "Determining distance with presence"
                elif (app_state == AppState.ESTIMATE_BREATHING_RATE 
                      and presence.distances_being_analyzed is not None
                      and len(presence.distances_being_analyzed) > 1):
                    start = presence.distances_being_analyzed[0]
                    end = presence.distances_being_analyzed[1]
                    s = slice(start, end)
                    distance_slice = distances[s]
                    start_m = "{:.2f}".format(distance_slice[0])
                    end_m = "{:.2f}".format(distance_slice[-1])
                    if ref_app_config.use_presence_processor:
                        presence.status = (
                            "Presence at distance " + start_m + "-" + end_m + "m"
                        )
                    else:
                        presence.status = "Presence distance detection disabled"
                elif app_state == AppState.INTRA_PRESENCE_DETECTED:
                    presence.status = "Large motion detected"
                else:
                    presence.status = ""
                    
        
                # breathing status
                if ref_app_result.breathing_result is not None:
                    breathing_result = ref_app_result.breathing_result.extra_result
                    breathing.time_vector = breathing_result.time_vector
                    breathing.motion = breathing_result.breathing_motion
                    breathing.psd = breathing_result.psd
                    breathing.psd_frequencies = breathing_result.frequencies
                    breathing.all_breathing_rate_history = breathing_result.all_breathing_rate_history
                    breathing.breathing_rate_history = breathing_result.breathing_rate_history
                    # if not np.isnan(breathing.breathing_rate_history[-1]):
                    #     breathing.breathing_rate = breathing_rate_history[-1]
                    # else:
                    #     breathing.breathing_rate = NaN
                    breathing.breathing_rate = ref_app_result.breathing_result.breathing_rate
                
                # breathing measurement status
                if app_state == AppState.ESTIMATE_BREATHING_RATE:
                    if (
                        ref_app_result.breathing_result is not None
                        and breathing.breathing_rate is None
                    ):
                        breathing.status = "Initializing respiration detection"
                    elif breathing.breathing_rate is not None:
                        breathing.status = f"Respiration rate: {breathing.breathing_rate:.1f} bpm"
                else:
                    breathing.status = "Waiting for distance"
                     
                     
                
                # Publish the data to the corresponding topics
                presence_pub.publish(presence.get_ROS_message())
                if (breathing.breathing_rate is not None 
                    and not (np.isnan(breathing.breathing_rate))):
                    #breathing_rate_pub.publish(breathing.breathing_rate)
                    breathing_pub.publish(breathing.get_ROS_message())
                    rate_db.append(breathing.breathing_rate)
                    if trigger.time > 15.0 and not trigger.sent:
                        trigger.reset()
                        msg = Float32()
                        msg.data = max(rate_db)
                        breathing_rate_pub.publish(msg)
                    if ((sensor_probing_counter * frequency_ros_log) % frequency_sensor_probing) == 0:
                        rospy.loginfo(f"RESPIRATION RATE: {breathing.breathing_rate:.1f}")
                elif trigger.time > 15.0 and not trigger.sent: 
                    trigger.reset()
                    msg = Float32()
                    msg.data = max(rate_db)
                    breathing_rate_pub.publish(msg)
                presence_status_pub.publish(presence.status)
                breathing_status_pub.publish(breathing.status)
                if ((sensor_probing_counter * frequency_ros_log) % frequency_sensor_probing) == 0:
                    rospy.loginfo(f"PRESENCE: {presence.status}; RESPIRATION: {breathing.status}")

                rate.sleep()
            except et.PGProccessDiedException:
                break
            except Exception as e:
                rospy.logerr(f"An unexpected error occurred: {e}")
                break

    ref_app.stop()
    client.close()

    

if __name__ == '__main__':
    try:
        sensor_data_publisher()
    except rospy.ROSInterruptException:
        pass
