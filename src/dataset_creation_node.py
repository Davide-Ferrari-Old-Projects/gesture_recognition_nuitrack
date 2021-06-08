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
rospy.init_node('dataset_creation_node')
package_path = rospkg.RosPack().get_path('gesture_recognition_nuitrack')
dataset_path = package_path + '/dataset/'
data_acquisition_path = dataset_path + 'data_acquisition/Filtered Data/'
# print(package_path)
print(dataset_path)


#####################################################################################################
#                                       Merge CSV Files                                             #
#####################################################################################################

# Read Database From the Following Set of .CSV Files
import csv

# Get First, Second, Third Rows
with open(data_acquisition_path + 'drop_dx_skeleton_pose.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',',)
    line_count = 0
    for row in csv_reader:
        if line_count == 0:
            # print(f'Column names are {", ".join(row)}')
            first_row = ['Gesture Label'] + row[4:]
            line_count += 1
        elif line_count == 1:
            # print(f'Column details are {", ".join(row)}')
            second_row = [' '] + row[4:]
            line_count += 1
        elif line_count == 2:
            # print(f'Column details are {", ".join(row)}')
            third_row = row
            break

with open(data_acquisition_path + 'drop_dx_skeleton_pose.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',',)
    line_count = 0
    for row in csv_reader:
        if line_count == 0 or line_count == 1 or line_count == 2:
            rospy.loginfo_once('Ignore First 3 Lines')
            # print('Ignore First 3 Lines')
            line_count += 1
        else:
            print(f'Data are: {", ".join(row)}')
            # print(f'\t{row[0]} works in the {row[1]} department, and was born in {row[2]}.')
            line_count += 1
            break
    print(f'Processed {line_count} lines.')


# Write add Data in a Single Dataset .CSV File
with open(dataset_path + 'dataset.csv', mode='w') as dataset:
    writer = csv.writer(dataset, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(first_row)
    writer.writerow(second_row)
    writer.writerow(third_row)


#####################################################################################################
#                                         Read Dataset                                              #
#####################################################################################################




# Gesture Labels
gesture_names = ['Point at DX', 'Point at SX', 'Take DX', 'Take SX', 'Drop DX', 'Drop SX']


