
Clone and build the repo. Then, it can run.

### BUILD
```
git clone https://github.com/jingjingccc/Eurobot2023_ws.git
mkdir ~/Eurobot2023_ws/src/YDLidar-SDK/build
cd ~/Eurobot2023_ws/src/YDLidar-SDK/build
cmake ..
make 
sudo make install
```
```
# at the root of the workspace
catkin_make
```

### RUN
```
# at the root of the workspace
source devel/setup.bash
roslaunch robot_navigation run.launch
```

(TO BE EDITED...)
