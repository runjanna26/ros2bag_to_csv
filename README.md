# ros2bag_to_csv
ros2 (foxy) package which convert ros2bag .db3 file to csv files.

## Download and install
```
cd /ros2_ws/src
git clone https://github.com/runjanna26/ros2bag_to_csv.git
```

## Build 
Use colcon to build package see on ([Colcon installation](https://colcon.readthedocs.io/en/released/user/installation.html))
```
sudo apt install python3-colcon-common-extensions
```
Change directory to ros2 workspace
```
cd /ros2_ws
colcon build
```

## How to use
1. Go to directory of ros2 bag file
2. Run

```
ros2 run ros2bag_to_csv ros2bag_to_csv
```
3. CSV files and folders will be generate to this directory

## Acknowledgments

I would like to express my gratitude to the owner of [fishros/ros2bag_convert](https://github.com/fishros/ros2bag_convert) for their invaluable contributions to the open-source community.

Thank you to [Owner's Name] for their dedication and hard work on this project.


## Version Record
- 20231222-V0.1.0
  - Completed basic conversion function
