#!/usr/bin/env python3

# Import TensorFlow, Keras
import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras.layers.experimental import preprocessing

# Import Panda, Numpy
import pandas as pd
import numpy as np

# Print("Import Libraries Succesful")
print("Tensorflow Version: " + tf.__version__ + "\n")

# Get Package Path
import rospy, rospkg
rospy.init_node('neural_network_node')
package_path = rospkg.RosPack().get_path('gesture_recognition_nuitrack')
dataset_path = package_path + '/dataset/'

#####################################################################################################
#                                         Read Dataset                                              #
#####################################################################################################


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

gesture_train.head()
gesture_features = gesture_train.copy()

# Gesture Labels ['Drop DX', 'Drop SX', 'Point At DX', 'Point At SX', 'Take DX', 'Take SX']
gesture_labels = gesture_features.pop('Gesture Name')

# Per questo set di dati tratterai tutte le funzionalità in modo identico. Comprimi le funzionalità in un singolo array NumPy:
gesture_features = np.array(gesture_features)
# gesture_features

# Data Normalization
normalize = preprocessing.Normalization()
normalize.adapt(gesture_features)

# Quindi crea un modello di regressione predire il gesto. Poiché esiste un solo tensore di input, qui è sufficiente un modello keras.Sequential
norm_gesture_model = tf.keras.Sequential([
  normalize,
  layers.Dense(128),
  layers.Dense(6)
])

# gesture_model = tf.keras.Sequential([
#   layers.Dense(64),
#   layers.Dense(1)
# ])

norm_gesture_model.compile(optimizer = tf.optimizers.Adam(),
                           loss = tf.losses.MeanSquaredError(),
                           metrics=['accuracy'])

# gesture_model.compile(loss = tf.losses.MeanSquaredError(),
#                       optimizer = tf.optimizers.Adam())

# Per addestrare quel modello, passa le caratteristiche e le etichette a Model.fit:
norm_gesture_model.fit(gesture_features, gesture_labels, epochs=20)
# gesture_model.fit(gesture_features, gesture_labels, epochs=10)

test_loss, test_acc = norm_gesture_model.evaluate(gesture_features,  gesture_labels, verbose=2)
print('\nTest accuracy:', test_acc)
