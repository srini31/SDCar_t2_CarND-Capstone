from styx_msgs.msg import TrafficLight
import rospy
import os
import yaml
import numpy as np
import cv2
import tensorflow as tf
from datetime import datetime

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        
        # traffic_light_config is parameter in ros/launch/site.launch and ros/launch/styx.launch
        #ros/launch/styx.launch - ros/src/tl_detector/sim_traffic_light_config.yaml - is_site: true
        #ros/launch/site.launch - ros/src/tl_detector/site_traffic_light_config.yaml - is_site: false
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.is_site = self.config["is_site"]
        rospy.logwarn("is_site: {0}".format(self.is_site))
        
        rospy.loginfo ("configuration file{}".format (self.config))
        #select & load appropriate model based environment configuration
        if (self.is_site):
            self.model_dir_path = 'light_classification/models/sim_styx.pb' #sim_model.h5
        else:
            self.model_dir_path = 'light_classification/models/site.pb' #site_model.h5
        
        
        #load the model
        if  not (os.path.exists(self.model_dir_path)):
            rospy.logerr ("model directory path {} does not exist".format (self.model_dir_path))
        #else:
        #    self.model = load_model(self.model_dir_path)
            
        self.graph = tf.Graph()
        self.threshold = .5
        
        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.model_dir_path, 'rb') as fid:
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')

        self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
        self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        self.scores = self.graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

        self.sess = tf.Session(graph=self.graph)

        
    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args: image (cv::Mat): image containing the traffic light
        Returns: int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction
        
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = datetime.now() #start = datetime.datetime.now() if import datetime
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})  
            end = datetime.now() #end = datetime.datetime.now()
            c = end - start
            #rospy.logwarn("tl_classifier - Image predicted in: {0} seconds".format(c.total_seconds()))
            #print(c.total_seconds())

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        print('tl_classifier - CLASSES: 1=Green, 2=Red, 3=Yellow, 4=Unknown: ', classes[0])
        #print('tl_classifier - SCORES: ', scores[0])
        #print('tl_classifier - TrafficLight.GREEN: ', TrafficLight.GREEN)     = 2  CLASSES: 1
        #print('tl_classifier - TrafficLight.RED: ', TrafficLight.RED)         = 0  CLASSES: 2
        #print('tl_classifier - TrafficLight.YELLOW: ', TrafficLight.YELLOW)   = 1  CLASSES: 3
        #print('tl_classifier - TrafficLight.UNKNOWN: ', TrafficLight.UNKNOWN) = 4  CLASSES: 4

        if scores[0] > self.threshold:
            if classes[0] == 1:
                print('GREEN')
                return TrafficLight.GREEN
            elif classes[0] == 2:
                print('RED')
                return TrafficLight.RED
            elif classes[0] == 3:
                print('YELLOW')
                return TrafficLight.YELLOW
            else:
                rospy.logwarn("Light: UNKNOWN")

       
        return TrafficLight.UNKNOWN
