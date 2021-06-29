#!/usr/bin/env python3

# Import TensorFlow, Keras
from numpy.core.fromnumeric import argmax
import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras.layers.experimental import preprocessing

# Import Panda, Numpy
import pandas as pd
import numpy as np

from rospy.rostime import Time

# Make numpy values easier to read.
np.set_printoptions(precision=3, suppress=True)

# Print("Import Libraries Succesful")
print("Tensorflow Version: " + tf.__version__ + "\n")

# Get Package Path
import rospy, rospkg
rospy.init_node('gesture_recognition_node')
package_path = rospkg.RosPack().get_path('gesture_recognition_nuitrack')
dataset_path = package_path + '/dataset/'
model_path = package_path + '/model/'

# Get the value of "train_network" variable from rosparam
try: train_network = rospy.get_param("/gesture_recognition_node/train_network")
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

    # Map Gesture Labels with Float ['Drop DX' = 0, 'Drop SX' = 1, 'Point At DX' = 2, 'Point At SX' = 3, 'Take DX' = 4, 'Take SX' = 5, 'No Gesture' = 6]
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
        elif gesture_labels[i] == "No Gesture":
            gesture_labels[i] = 6.0

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
        layers.Dense(7, activation = "softmax")
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
#                                     Compute PointAt Direction                                      #
######################################################################################################

# Equazioni parametriche di una retta nello spazio con un vettore e un punto
# https://www.youmath.it/lezioni/algebra-lineare/geometria-dello-spazio/677-formule-per-le-equazioni-parametriche-della-retta.html

from geometry_msgs.msg import Point
from skspatial.objects import Line
from skspatial.objects import Points

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
        
    # Create Points Vector
    points = Points([
        [shoulder.x, shoulder.y, shoulder.z],
        [elbow.x, elbow.y, elbow.z],
        [wrist.x, wrist.y, wrist.z],
        [hand.x, hand.y, hand.z]
    ])
    
    # Compute Line Best Fitting
    line_fit = Line.best_fit(points)

    return line_fit


######################################################################################################
#                                    ROS PUBLISHER INITIALIZATION                                    #
######################################################################################################

from nuitrack_msgs.msg import SkeletonDataArray
from gesture_recognition_nuitrack.msg import gesture_recognized_array, gesture_recognized, gesture_probability, pointat_direction

# Gesture Vector Publisher
gesture_publisher = rospy.Publisher('/gesture_recognition/gesture', gesture_recognized, queue_size=1)
gesture_mean_publisher = rospy.Publisher('/gesture_recognition/gesture_probability_mean', gesture_recognized, queue_size=1)
gesture_vector_publisher = rospy.Publisher('/gesture_recognition/gesture_vector', gesture_recognized_array, queue_size=1)

gesture_vector = gesture_recognized_array()
gesture_vector.header.frame_id = 'Gesture Recognized in Last 2 Seconds'
timer = rospy.Time.now()

def probability_filter(gesture_vector):

    # Gesture Mean Vector
    gesture_mean = gesture_recognized()
    gesture_mean.header.frame_id = 'Gesture 2s Probability Mean'
    gesture_mean.gesture_names = ['Drop DX', 'Drop SX', 'Point At DX', 'Point At SX', 'Take DX', 'Take SX']

    gesture_probability_counter = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    
    for i in range (0, len(gesture_vector.gesture_recognized)):

        # Sum each No Gesture Probability Values
        gesture_probability_counter[len(gesture_probability_counter)] += gesture_vector.gesture_recognized[i].no_gesture_probability;

        for j in range (0, len(gesture_vector.gesture_recognized[i].gesture_probability)):

            # Sum the probability for each gesture in the array
            gesture_probability_counter[j] += gesture_vector.gesture_recognized[i].gesture_probability[j];
        
    # Divide each voice for the number of gestures received and save in Gesture Probability Vector
    for i in range (0, len(gesture_probability_counter)): gesture_probability_counter = gesture_probability_counter[i] / len(gesture_vector.gesture_recognized)

    # Check if No Gesture is the more Probabile Result
    if   gesture_probability_counter.index(max(gesture_probability_counter)) == 0: gesture = "Drop DX"
    elif gesture_probability_counter.index(max(gesture_probability_counter)) == 1: gesture = "Drop SX"
    elif gesture_probability_counter.index(max(gesture_probability_counter)) == 2: gesture = "Point At DX"
    elif gesture_probability_counter.index(max(gesture_probability_counter)) == 3: gesture = "Point At SX"
    elif gesture_probability_counter.index(max(gesture_probability_counter)) == 4: gesture = "Take DX"
    elif gesture_probability_counter.index(max(gesture_probability_counter)) == 5: gesture = "Take SX"
    elif gesture_probability_counter.index(max(gesture_probability_counter)) == 6: gesture = "No Gesture"

    # Assign Mean Value to gesture_mean Message
    for i in range (0, len(gesture_mean.gesture_names)): gesture_mean.gesture_probability[i] = gesture_probability_counter[i]
    gesture_mean.no_gesture_probability = gesture_probability_counter[len(gesture_probability_counter)]

    # Compute Point At Direction Mean
    if (gesture == 'Point At DX' or gesture == 'Point At SX'):

        direction_mean = pointat_direction()
        pointat_counter = 0

        # Check all Gesture Vector
        for i in range (0, len(gesture_vector.gesture_recognized)):

            # Compute only the direction when gesture is "Point At DX" or "Point At SX"
            if gesture_vector.gesture_recognized[i].gesture_probability.index(max(gesture_vector.gesture_recognized[i].gesture_probability)) == 2 or gesture_vector.gesture_recognized[i].gesture_probability.index(max(gesture_vector.gesture_recognized[i].gesture_probability)) == 3:
                
                # Increase Counter
                pointat_counter += 1

                # Sum each Point 
                direction_mean.point.x += gesture_vector.gesture_recognized[i].pointat_direction.point.x
                direction_mean.point.y += gesture_vector.gesture_recognized[i].pointat_direction.point.y
                direction_mean.point.z += gesture_vector.gesture_recognized[i].pointat_direction.point.z
                
                # Sum each Direction Vector
                for i in range (0,3): direction_mean.direction_vector[i] += gesture_vector.gesture_recognized[i].pointat_direction.direction_vector[i]

        # Divide Summation for Number of Point At Gestures    
        direction_mean.point.x *= 1 / float(pointat_counter)
        direction_mean.point.y *= 1 / float(pointat_counter)
        direction_mean.point.z *= 1 / float(pointat_counter)
        for i in range (0,3): direction_mean.direction_vector[i] *= 1 / float(pointat_counter)
        
        # Assign Mean to ROS Message
        gesture_mean.pointat_direction.point = direction_mean.point
        gesture_mean.pointat_direction.direction_vector = direction_mean.direction_vector

    if (gesture != "No Gesture"):

        # Print Prevision Gesture
        rospy.logwarn_throttle(2, 'Prevision: Gesture ' + str(gesture_probability_counter.index(max(gesture_probability_counter))) + ' || Name: ' + gesture)
        rospy.loginfo_throttle(2, str(gesture_probability_counter) + '\n')

        # Print the Direction of the Gesture (if present)
        if gesture == "Point At DX" or gesture == "Point At SX": rospy.loginfo_throttle(2, 'Point At Direction || Line Point: ' + str(gesture_mean.pointat_direction.point) + ' || Direction Vector: ' + str(gesture_mean.pointat_direction.direction_vector) + '\n')

        # Publish Mean Gesture Message
        gesture_mean_publisher.publish(gesture_mean)

    else: rospy.logwarn_throttle(2, 'No Gesture Recognized\n')

######################################################################################################
#                                        NUITRACK CALLBACK                                           #
######################################################################################################

# Debug Print Variable
print_prevision = False

def nuitrack_callback (skeleton_message):
    
    # Convert the nuitrack message to array (60 float coordinates x,y,z)
    if (len(skeleton_message.skeletons) > 0):

        # Skeleton Data Array
        skeleton_data = []
        
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

        # Rename Prevision
        if   argmax(prevision,axis = 0) == 0.0: gesture = "Drop DX"
        elif argmax(prevision,axis = 0) == 1.0: gesture = "Drop SX"
        elif argmax(prevision,axis = 0) == 2.0: gesture = "Point At DX"
        elif argmax(prevision,axis = 0) == 3.0: gesture = "Point At SX"
        elif argmax(prevision,axis = 0) == 4.0: gesture = "Take DX"
        elif argmax(prevision,axis = 0) == 5.0: gesture = "Take SX"
        elif argmax(prevision,axis = 0) == 6.0: gesture = "No Gesture"

        # Compute Point At Direction (if present)
        if   gesture == "Point At DX": direction = pointat_compute_direction("Point At DX", skeleton_message.skeletons[0])
        elif gesture == "Point At SX": direction = pointat_compute_direction("Point At SX", skeleton_message.skeletons[0])

        # DEBUG - Print Prevision (TOLERANCE = 0.98)
        global print_prevision
        if (print_prevision and max(prevision) > 0.98):
            
            rospy.logwarn_throttle(2, 'Prevision: Gesture ' + str(argmax(prevision,axis = 0)) + ' || Name: ' + gesture)
            rospy.loginfo_throttle(2, str(prevision) + '\n')

            # Print the Direction of the Gesture (if present)
            if gesture == "Point At DX" or gesture == "Point At SX": rospy.loginfo_throttle(2, 'Point At Direction || Line Point: ' + str(direction.point) + ' || Direction Vector: ' + str(direction.direction) + '\n')


        # Create Gesture Message
        gesture_message = gesture_recognized()
        gesture_message.header.stamp = rospy.Time.now()

        # Save Prevision in the Gesture Vector
        gesture_message.gesture_names = ['Drop DX', 'Drop SX', 'Point At DX', 'Point At SX', 'Take DX', 'Take SX']
        for i in range(0, len(gesture_message.gesture_names)): gesture_message.gesture_probability[i] = float(prevision[i])
        gesture_message.no_gesture_probability = float(prevision[6])

        # Save PointAt Direction in the Gesture Vector
        if gesture == "Point At DX" or gesture == "Point At SX":
            
            gesture_message.pointat_direction.point.x = direction.point[0]
            gesture_message.pointat_direction.point.y = direction.point[1]
            gesture_message.pointat_direction.point.z = direction.point[2]
            gesture_message.pointat_direction.direction_vector = direction.direction

            # DEBUG
            # print ('\nPointAt Point:', end=" ")
            # print(gesture_message.pointat_direction.point)
            # print ('\nPointAt Direction:', end=" ")
            # print(gesture_message.pointat_direction.direction_vector)

        # Append Message in the Gesture Vector
        gesture_vector.gesture_recognized.append(gesture_message)

        # Remove Old Messages from Gesture Vector
        while (rospy.Time.now() - gesture_vector.gesture_recognized[0].header.stamp).to_sec() >= 2: gesture_vector.gesture_recognized.pop(0)

        # Publish Single Messages (Every Cycle)
        gesture_publisher.publish(gesture_message)

        # Publish Array Message (Every 1/2 Second)
        global timer
        if (rospy.Time.now() - timer).to_sec() >= 0.5: 
            gesture_vector.header.stamp = rospy.Time.now()
            gesture_vector_publisher.publish(gesture_vector)
            timer = rospy.Time.now()

        # Compute the Probabilistic Mean of the Gestures and Publishes the Result
        probability_filter(gesture_vector)
        

    else: rospy.logerr_throttle(5,"Neural Network Node: Skeleton Message Missing")


######################################################################################################
#                                                MAIN                                                #
######################################################################################################

while not rospy.is_shutdown():
    
    rospy.Subscriber('/nuitrack/skeletons', SkeletonDataArray, nuitrack_callback)

    rospy.spin()
