# gesture_recognition_nuitrack

Package that allows gesture recognition with Nuitrack-SDK. There are 3 main functions:

- data_acquisition      →  Read data from Nuitrack-ROS and save skeleton points into a .CSV database
- dataset_creation      →  Read each .CSV data and create a dataset
- gesture_recognition   →  Train a neural network and use it to predict the gesture

## Requirements

* Ubuntu 18.04+
* Python 3.6+
* ROS Melodic+

## Installation

* Navigate to source directory of your ROS catkin workspace (e.g. `catkin_ws`):

  ```
  cd catkin_ws/src
  copy the package (git clone or others)
  ```

* Build catkin workspace:

  ```
  cd catkin_ws
  catkin_make
  ```
  
* Source workspace:

  ```
  source catkin_ws/devel/setup.bash
  ```
&nbsp;


# data_acquisition

To use data acquisition node the best thing you can do is to record a short rosbag video with the nuitrack-ros sdk for each gesture you want to add in the dataset.

1. Start `Nuitrack-ROS SDK` Node and record a rosbag for each gesture:

        rosbag record --split --size=5000 --buffsize=0 -a

2. Filter the contents of the bag in order to create a new bag wich contains only the desired gesture:

        rosbag filter input.bag output.bag "t.to_sec() <= 18615.43"

3. Open the `data_acquisition` node and call the `start_registration` service; the message data field contains the desired .csv name (gesture name):
   
        rosservice call /data_acquisition_Node/start_registration "message_data: 'my_gesture'"

4. Play the filtered rosbag. For each rosbag frame analyzed by nuitrack, the node draws the 2D skeleton over the rgb camera image. If a skeleton position is close enough to that of the previous frame it is automatically saved, otherwise the still image will be shown to the user who will have to decide whether to accept it or discard it by pressing the 's' and 'n' keys respectively on the keyboard (on the image window).

5. Call the `stop_registration` trigger service to end the registration session.

        rosservice call /data_acquisition_Node/stop_registration "{}"

&nbsp;

# dataset_creation

Python script that merge each gesture dataset in a single file.

    roslaunch gesture_recognition_nuitrack dataset_creation.launch


# gesture_recognition

Neural Network Gesture Recognition Node that allows to train a NN on the previously created dataset and use that model to predict the gesture analyzing the skeleton points received with the Nuitrack-ROS Node

1. Start the node with the arguments `train_network:=true` in order to start NN Training. (Needed only in first execution)

2. Read on topic / on terminal the prevision results

