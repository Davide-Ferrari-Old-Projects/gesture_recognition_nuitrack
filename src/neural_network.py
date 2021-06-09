#!/usr/bin/env python3

# Import TensorFlow, Keras
from numpy.core.fromnumeric import argmax
import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras.layers.experimental import preprocessing

# Import Panda, Numpy
import pandas as pd
import numpy as np
import random

# Make numpy values easier to read.
np.set_printoptions(precision=3, suppress=True)

# Print("Import Libraries Succesful")
print("Tensorflow Version: " + tf.__version__ + "\n")

# Get Package Path
import rospy, rospkg
rospy.init_node('neural_network_node')
package_path = rospkg.RosPack().get_path('gesture_recognition_nuitrack')
dataset_path = package_path + '/dataset/'


######################################################################################################
#                                        Train Neural Network                                        #
######################################################################################################

# Read Dataset from .CSV
gesture_train = pd.read_csv(
    dataset_path + 'train_dataset.csv',
    names=["Gesture Name", "joint_head_x", "joint_head_y", "joint_head_z", "joint_neck_x", "joint_neck_y", "joint_neck_z", "joint_torso_x", "joint_torso_y", "joint_torso_z", "joint_waist_x", "joint_waist_y", "joint_waist_z"
                         , "joint_left_collar_x", "joint_left_collar_y", "joint_left_collar_z", "joint_left_shoulder_x", "joint_left_shoulder_y", "joint_left_shoulder_z", "joint_left_elbow_x", "joint_left_elbow_y", "joint_left_elbow_z"
                         , "joint_left_wrist_x", "joint_left_wrist_y", "joint_left_wrist_z", "joint_left_hand_x", "joint_left_hand_y", "joint_left_hand_z"
                         , "joint_right_collar_x", "joint_right_collar_y", "joint_right_collar_z", "joint_right_shoulder_x", "joint_right_shoulder_y", "joint_right_shoulder_z", "joint_right_elbow_x", "joint_right_elbow_y", "joint_right_elbow_z"
                         , "joint_right_wrist_x", "joint_right_wrist_y", "joint_right_wrist_z", "joint_right_hand_x", "joint_right_hand_y", "joint_right_hand_z"
                         , "joint_left_hip_x", "joint_left_hip_y", "joint_left_hip_z", "joint_left_knee_x", "joint_left_knee_y", "joint_left_knee_z", "joint_left_ankle_x", "joint_left_ankle_y", "joint_left_ankle_z"
                         , "joint_right_hip_x", "joint_right_hip_y", "joint_right_hip_z", "joint_right_knee_x", "joint_right_knee_y", "joint_right_knee_z", "joint_right_ankle_x", "joint_right_ankle_y", "joint_right_ankle_z"]
)

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
print('Gesture Feature:\n')
print(gesture_features)
print('Gesture Feature Shape:\n')
print(gesture_features.shape)
print('Gesture Labels:\n')
print(gesture_labels)
print('Gesture Labels Shape:\n')
print(gesture_labels.shape)
print('\n')

# Creation of the Neural Networks Model (keras.Sequential)
gesture_model = tf.keras.Sequential([
      
      # Two Initial Layers ("relu" = rectified linear unit)
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

# Train the Model
gesture_model.fit(gesture_features, gesture_labels, epochs = 50)

######################################################################################################
#                                        Test Neural Network                                         #
######################################################################################################

random_test = 10

for i in range(0,random_test):

      random_index=random.randint(0,len(gesture_labels))
      real=gesture_labels[random_index]
      print("Real")
      print(real.numpy())
      print("Predicted")
      input= gesture_features[random_index]
      input = tf.convert_to_tensor(input,dtype=tf.float32)
      input=tf.expand_dims(input,axis=0)
      prevision = gesture_model(input).numpy()[0]

      print(argmax(prevision,axis=0))
      print(prevision)
#     result = np.where(prevision == np.amax(prevision))
#     #print(gesture_model(input).numpy())
#     print(result)
      print("------------------")


#test_loss, test_acc = gesture_model.evaluate(gesture_features,  gesture_labels, verbose=0)
#print('\nTest accuracy:', test_acc)
