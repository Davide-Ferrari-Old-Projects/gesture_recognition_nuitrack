#!/usr/bin/env python3

# Import TensorFlow, Keras
from numpy.core.fromnumeric import argmax
import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras.layers.experimental import preprocessing

# Import Panda, Numpy
import pandas as pd
import numpy as np

# Make numpy values easier to read.
np.set_printoptions(precision=3, suppress=True)

# Print("Import Libraries Succesful")
print("Tensorflow Version: " + tf.__version__ + "\n")

# Get Package Path
import rospy, rospkg
rospy.init_node('neural_network_node')
package_path = rospkg.RosPack().get_path('gesture_recognition_nuitrack')
dataset_path = package_path + '/dataset/'
model_path = package_path + '/model/'

# Get the value of "train_network" variable from rosparam
try: train_network = rospy.get_param("/neural_network_node/train_network")
except: train_network = False

######################################################################################################
#                                        Train Neural Network                                        #
######################################################################################################

if (train_network):

    # Read Dataset from .CSV
    gesture_train = pd.read_csv(
    dataset_path + 'dataset.csv',
    names=["Gesture Name", "joint_head_x", "joint_head_y", "joint_head_z", "joint_neck_x", "joint_neck_y", "joint_neck_z", "joint_torso_x", "joint_torso_y", "joint_torso_z", "joint_waist_x", "joint_waist_y", "joint_waist_z"
                         , "joint_left_collar_x", "joint_left_collar_y", "joint_left_collar_z", "joint_left_shoulder_x", "joint_left_shoulder_y", "joint_left_shoulder_z", "joint_left_elbow_x", "joint_left_elbow_y", "joint_left_elbow_z"
                         , "joint_left_wrist_x", "joint_left_wrist_y", "joint_left_wrist_z", "joint_left_hand_x", "joint_left_hand_y", "joint_left_hand_z"
                         , "joint_right_collar_x", "joint_right_collar_y", "joint_right_collar_z", "joint_right_shoulder_x", "joint_right_shoulder_y", "joint_right_shoulder_z", "joint_right_elbow_x", "joint_right_elbow_y", "joint_right_elbow_z"
                         , "joint_right_wrist_x", "joint_right_wrist_y", "joint_right_wrist_z", "joint_right_hand_x", "joint_right_hand_y", "joint_right_hand_z"
                         , "joint_left_hip_x", "joint_left_hip_y", "joint_left_hip_z", "joint_left_knee_x", "joint_left_knee_y", "joint_left_knee_z", "joint_left_ankle_x", "joint_left_ankle_y", "joint_left_ankle_z"
                         , "joint_right_hip_x", "joint_right_hip_y", "joint_right_hip_z", "joint_right_knee_x", "joint_right_knee_y", "joint_right_knee_z", "joint_right_ankle_x", "joint_right_ankle_y", "joint_right_ankle_z"]
    )

    # Randomize Dataset
    gesture_randomized = gesture_train.sample(frac = 1)
    gesture_randomized.to_csv(dataset_path + 'gesture_randomized.csv')
    gesture_train = gesture_randomized

    # Print first 5 rows to show if the reading was succesfull
    print('Dataset Head (First 5 Rows):\n')
    print(gesture_train.head())
    print('\n')

    # Copy the dataset into a feature vector and pop the names in a label vector
    gesture_features = gesture_train.copy()
    gesture_labels = gesture_features.pop('Gesture Name')

    # Map Gesture Labels with Float ['Drop DX' = 0, 'Drop SX' = 1, 'Point At DX' = 2, 'Point At SX' = 3, 'Take DX' = 4, 'Take SX' = 5]
    for i in range (0,len(gesture_labels)):
        if   gesture_labels[i] == "Drop DX":
            gesture_labels[i] = 0.0
        elif gesture_labels[i] == "Drop SX":
            gesture_labels[i] = 1.0
        elif gesture_labels[i] == "Point At DX":
            gesture_labels[i] = 2.0
        elif gesture_labels[i] == "Point At SX":
            gesture_labels[i] = 3.0
        elif gesture_labels[i] == "Take DX":
            gesture_labels[i] = 4.0
        elif gesture_labels[i] == "Take SX":
            gesture_labels[i] = 5.0

    # Print Gesture Label Column
    print('Gesture Label Column:\n')
    print(gesture_labels)
    print('\n')

    # Convert "gesture_features" and "gesture_labels" first in Array and next in Tensor
    gesture_features = np.array(gesture_features)
    gesture_features = tf.convert_to_tensor(gesture_features,dtype=tf.float32)
    gesture_labels = np.array(gesture_labels)
    gesture_labels = tf.convert_to_tensor(gesture_labels,dtype=tf.float32)

    # Print Gesture Labels and Features and their Shape
    print('\nGesture Feature:\n')
    print(gesture_features)
    print('\nGesture Labels:\n')
    print(gesture_labels)
    print('\n')

    # Creation of the Neural Networks Model (keras.Sequential)
    gesture_model = tf.keras.Sequential([
        
        # Four Initial Layers ("relu" = rectified linear unit)
        layers.Dense(512, activation = "relu"),
        layers.Dense(256, activation = "relu"),
        
        # Stochastic Interruptions in the Training Phase to Avoid Overfitting
        layers.Dropout(0.1),
        
        # Other "relu" Layers
        layers.Dense(256, activation = "relu"),
        layers.Dense(256, activation = "relu"),
        layers.Dense(128, activation = "relu"),
        layers.Dense(128, activation = "relu"),
        
        # Output Layer ("softmax") to Match the "SparseCategoricalCrossentropy" Loss Function
        layers.Dense(6, activation = "softmax")
    ])

    # Instructions for Filling in the Model
    gesture_model.compile(
        loss = tf.keras.losses.SparseCategoricalCrossentropy(from_logits = True),
        optimizer = tf.keras.optimizers.Adam(),
        metrics = [tf.keras.metrics.SparseCategoricalAccuracy()]
    )

    # Train The Model
    gesture_model.fit(gesture_features, gesture_labels, epochs = 50, validation_split = 0.3)

    # Test The Model
    import random
    random_test=10

    for i in range(0,random_test):

        random_index=random.randint(0,len(gesture_labels))
        real=gesture_labels[random_index]
        print("Real Gesture: " + str(real.numpy()))
        input= gesture_features[random_index]
        input = tf.convert_to_tensor(input,dtype=tf.float32)
        input=tf.expand_dims(input,axis=0)
        prevision = gesture_model(input).numpy()[0]
        print("Predicted Gesture: " + str(argmax(prevision,axis = 0)))
        print(prevision)
        print('\n')

    # Save the Model
    gesture_model.save(model_path + 'neural_network_model.h5')
    print('Model Saved')


######################################################################################################
#                                        Load Neural Network                                         #
######################################################################################################

else:

    # Load the Pre-Trained Model
    gesture_model = tf.keras.models.load_model(model_path + 'neural_network_model.h5')

    # Show the model architecture
    gesture_model.summary()
    print('\n')
    rospy.logwarn('Neural Network Loaded\n')


######################################################################################################
#                                     COMPUTE POINTAT DIRECTION                                      #
######################################################################################################

from geometry_msgs.msg import Point
from skspatial.objects import Line
from skspatial.objects import Points

begin = rospy.Time.now()

def pointat_compute_direction(pointat_dx_sx, skeleton):

    if (pointat_dx_sx == 'Point At DX'):

        SHOULDER = "joint_right_shoulder"
        ELBOW = "joint_right_elbow"
        WRIST = "joint_right_wrist"
        HAND = "joint_right_hand"

    elif (pointat_dx_sx == 'Point At SX'):

        SHOULDER = "joint_left_shoulder"
        ELBOW = "joint_left_elbow"
        WRIST = "joint_left_wrist"
        HAND = "joint_left_hand"

    # Define the Arm Joints
    shoulder = Point()
    elbow = Point()
    wrist = Point()
    hand = Point()
    
    for i in range (0,len(skeleton.joint_names)):

        if skeleton.joint_names[i]   == SHOULDER: shoulder = skeleton.joint_pos_3D[i]
        elif skeleton.joint_names[i] == ELBOW:    elbow = skeleton.joint_pos_3D[i]
        elif skeleton.joint_names[i] == WRIST:    wrist = skeleton.joint_pos_3D[i]
        elif skeleton.joint_names[i] == HAND:     hand = skeleton.joint_pos_3D[i]
    
    # DEBUG
    # rospy.loginfo_throttle(2, pointat_dx_sx + '\t|| Joint Names: ' + SHOULDER + ' | ' + ELBOW + ' | '  + WRIST + ' | '  + HAND)
    # rospy.loginfo_throttle(2, 'Shoulder' + '\t|| x: ' + '%.8f' % shoulder.x + '\ty: ' + '%.8f' % shoulder.y + '\tz: ' + '%.8f' % shoulder.z)
    # rospy.loginfo_throttle(2, 'Elbow'    + '\t|| x: ' + '%.8f' % elbow.x    + '\ty: ' + '%.8f' % elbow.y    + '\tz: ' + '%.8f' % elbow.z)
    # rospy.loginfo_throttle(2, 'Wrist'    + '\t|| x: ' + '%.8f' % wrist.x    + '\ty: ' + '%.8f' % wrist.y    + '\tz: ' + '%.8f' % wrist.z)
    # rospy.loginfo_throttle(2, 'Hand'     + '\t|| x: ' + '%.8f' % hand.x     + '\ty: ' + '%.8f' % hand.y     + '\tz: ' + '%.8f' % hand.z)


    global begin

    # Compute the Direction of the Gesture
    if ((rospy.Time.now() - begin).to_sec() > 2):
        
        # Create Points Vector
        points = Points([
            [shoulder.x, shoulder.y, shoulder.z],
            [elbow.x, elbow.y, elbow.z],
            [wrist.x, wrist.y, wrist.z],
            [hand.x, hand.y, hand.z]
        ])
        
        # Compute Line Best Fitting
        line_fit = Line.best_fit(points)
        print('Point At Direction || Line Point: ' + str(line_fit.point) + ' || Direction Vector: ' + str(line_fit.direction) + '\n')

        # Reset Begin Time
        begin = rospy.Time.now()
        

######################################################################################################
#                                             CALLBACKS                                              #
######################################################################################################

from nuitrack_msgs.msg import SkeletonDataArray

def nuitrack_callback (skeleton_message):

    # Skeleton Data Array
    skeleton_data = []
    
    # Convert the nuitrack message to array (60 float coordinates x,y,z)
    if (len(skeleton_message.skeletons) > 0):
        
        for i in range(0,len(skeleton_message.skeletons[0].joint_pos_3D)):
            skeleton_data.append(skeleton_message.skeletons[0].joint_pos_3D[i].x)
            skeleton_data.append(skeleton_message.skeletons[0].joint_pos_3D[i].y)
            skeleton_data.append(skeleton_message.skeletons[0].joint_pos_3D[i].z)

        # print(skeleton_data)
    
    # Convert Skeleton Data in Tensor
    input = np.array(skeleton_data)
    input = tf.convert_to_tensor(input, dtype = tf.float32)
    input = tf.expand_dims(input, axis = 0)

    # Get Gesture Prevision (Pass Gesture to Neural Network)
    prevision = gesture_model(input).numpy()[0]

    # Print Prevision (TOLERANCE = 0.9)
    if (max(prevision) > 0.9):

        if   argmax(prevision,axis = 0) == 0.0: gesture = "Drop DX"
        elif argmax(prevision,axis = 0) == 1.0: gesture = "Drop SX"
        elif argmax(prevision,axis = 0) == 2.0: gesture = "Point At DX"
        elif argmax(prevision,axis = 0) == 3.0: gesture = "Point At SX"
        elif argmax(prevision,axis = 0) == 4.0: gesture = "Take DX"
        elif argmax(prevision,axis = 0) == 5.0: gesture = "Take SX"

        rospy.logwarn_throttle(2, 'Prevision: Gesture ' + str(argmax(prevision,axis = 0)) + ' || Name: ' + gesture)
        rospy.loginfo_throttle(2, str(prevision) + '\n')

        # Compute Point At Direction
        if   gesture == "Point At DX": pointat_compute_direction("Point At DX", skeleton_message.skeletons[0])
        elif gesture == "Point At SX": pointat_compute_direction("Point At SX", skeleton_message.skeletons[0])


######################################################################################################
#                                                MAIN                                                #
######################################################################################################

while not rospy.is_shutdown():
    
    rospy.Subscriber('/nuitrack/skeletons', SkeletonDataArray, nuitrack_callback)

    rospy.spin()
