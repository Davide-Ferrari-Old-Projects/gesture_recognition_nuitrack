#!/usr/bin/env python3
import rospy, rospkg
rospy.init_node('dataset_creation_node', disable_signals=True)

# Get Package Path
package_path = rospkg.RosPack().get_path('gesture_recognition_nuitrack')
dataset_path = package_path + '/dataset/'
data_acquisition_path = dataset_path + 'data_acquisition/Filtered Data/'
# print(package_path)
print('\nDataset Path: ' + dataset_path + '\n')


#####################################################################################################
#                                          Merge CSV Files                                          #
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

# Drop DX Data
drop_dx_data = []

with open(data_acquisition_path + 'drop_dx_skeleton_pose.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',',)
    rospy.logwarn_once('Opening Drop DX data file.')
    line_count = 0
    for row in csv_reader:
        if line_count == 0 or line_count == 1 or line_count == 2:
            rospy.loginfo_once('Ignore First 3 Lines')
            # print('Ignore First 3 Lines')
            line_count += 1
        else:
            drop_dx_data.append(['Drop DX'] + row[4:])
            # print(drop_dx_data)
            # print(f'Data are: {", ".join(row)}')
            line_count += 1
    rospy.loginfo(f'Processed {line_count} lines.\n')

# Drop SX Data
drop_sx_data = []

with open(data_acquisition_path + 'drop_sx_skeleton_pose.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',',)
    rospy.logwarn_once('Opening Drop SX data file.')
    line_count = 0
    for row in csv_reader:
        if line_count == 0 or line_count == 1 or line_count == 2:
            rospy.loginfo_once('Ignore First 3 Lines')
            # print('Ignore First 3 Lines')
            line_count += 1
        else:
            drop_sx_data.append(['Drop SX'] + row[4:])
            # print(drop_sx_data)
            # print(f'Data are: {", ".join(row)}')
            line_count += 1
    rospy.loginfo(f'Processed {line_count} lines.\n')

# Point At Dx Data
pointat_dx_data = []

with open(data_acquisition_path + 'pointat_dx_skeleton_pose.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',',)
    rospy.logwarn_once('Opening Point At DX data file.')
    line_count = 0
    for row in csv_reader:
        if line_count == 0 or line_count == 1 or line_count == 2:
            rospy.loginfo_once('Ignore First 3 Lines')
            # print('Ignore First 3 Lines')
            line_count += 1
        else:
            pointat_dx_data.append(['Point At DX'] + row[4:])
            # print(pointat_dx_data)
            # print(f'Data are: {", ".join(row)}')
            line_count += 1
    rospy.loginfo(f'Processed {line_count} lines.\n')

# Point At Sx Data
pointat_sx_data = []

with open(data_acquisition_path + 'pointat_sx_skeleton_pose.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',',)
    rospy.logwarn_once('Opening Point At SX data file.')
    line_count = 0
    for row in csv_reader:
        if line_count == 0 or line_count == 1 or line_count == 2:
            rospy.loginfo_once('Ignore First 3 Lines')
            # print('Ignore First 3 Lines')
            line_count += 1
        else:
            pointat_sx_data.append(['Point At SX'] + row[4:])
            # print(pointat_sx_data)
            # print(f'Data are: {", ".join(row)}')
            line_count += 1
    rospy.loginfo(f'Processed {line_count} lines.\n')

# Take Dx Data
take_dx_data = []

with open(data_acquisition_path + 'take_dx_skeleton_pose.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',',)
    rospy.logwarn_once('Opening Take DX data file.')
    line_count = 0
    for row in csv_reader:
        if line_count == 0 or line_count == 1 or line_count == 2:
            rospy.loginfo_once('Ignore First 3 Lines')
            # print('Ignore First 3 Lines')
            line_count += 1
        else:
            take_dx_data.append(['Take DX'] + row[4:])
            # print(take_dx_data)
            # print(f'Data are: {", ".join(row)}')
            line_count += 1
    rospy.loginfo(f'Processed {line_count} lines.\n')

# Take Sx Data
take_sx_data = []

with open(data_acquisition_path + 'take_sx_skeleton_pose.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',',)
    rospy.logwarn_once('Opening Take SX data file.')
    line_count = 0
    for row in csv_reader:
        if line_count == 0 or line_count == 1 or line_count == 2:
            rospy.loginfo_once('Ignore First 3 Lines')
            # print('Ignore First 3 Lines')
            line_count += 1
        else:
            take_sx_data.append(['Take SX'] + row[4:])
            # print(take_sx_data)
            # print(f'Data are: {", ".join(row)}')
            line_count += 1
    rospy.loginfo(f'Processed {line_count} lines.\n')

# Write all Data in a Single Dataset .CSV File
with open(dataset_path + 'dataset.csv', mode='w') as dataset:
    writer = csv.writer(dataset, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    
    # Write Dataset Intestation
    # writer.writerow(first_row)
    # writer.writerow(second_row)
    # writer.writerow(third_row)
    
    # Write Dataset Data
    for element in drop_dx_data: writer.writerow(element)
    for element in drop_sx_data: writer.writerow(element)
    for element in pointat_dx_data: writer.writerow(element)
    for element in pointat_sx_data: writer.writerow(element)
    for element in take_dx_data: writer.writerow(element)
    for element in take_sx_data: writer.writerow(element)

    rospy.logwarn_once('Dataset Created\n')

rospy.signal_shutdown("Program Ended")
