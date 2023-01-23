
Clone and build the repo. Then, it can run.


```
git clone --recursive https://github.com/jingjingccc/Eurobot2023_ws.git 
```

### INSTALL
```
sudo apt-get install ros-{distro}-rosserial ros-{distro}-rosserial-arduino
sudo apt-get install ros-{distro}-costmap-converter
sudo apt-get install ros-{distro}-navigation
sudo apt-get install build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev cmake
```

### BUILD
#### [YDLiDAR]
```
mkdir ~/Eurobot2023_ws/src/YDLidar-SDK/build
cd ~/Eurobot2023_ws/src/YDLidar-SDK/build
cmake ..
make 
sudo make install
```
#### [libsurvive]
```
cd ~/Eurobot2023_ws/src/libsurvive
make
```
```
# at the root of the workspace
catkin_make
```

### SETTING
#### [vive lighthouse / tracker permission]
```
sudo cp -r 81-vive.rules /etc/udev/rules.d
```
#### [rename RPI USB ports]
originated from : @sunfu-chou
```
chmod +x ./rename_RPI_USB_ports.sh
sudo ./rename_RPI_USB_ports.sh
```
```
# check USB port
ls -l /dev | grep ttyUSB
```
<img src="img/RPI_USB.png" height="200">

### RUN
```
# at the root of the workspace
source devel/setup.bash
roslaunch robot_navigation run.launch
```

(TO BE EDITED...)
