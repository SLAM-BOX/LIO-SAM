# LIO-SAM 

## For Lio-sam

***Install this repo***
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/SLAM-BOX/LIO-SAM.git --branch dev
cd ../../
catkin build
```

***Run lio-sam***
For real data
```bash
roslaunch lio-sam run.launch
```

For lgsvl data, we need to add ring channel into the /velodyne_points topic.
```bash
roscd lgsvl && python convert.py
```

Run lio-sam on lgsvl
```bash
roslaunch lio-sam run_lgsvl.launch
```

### Prepare data
链接：https://pan.baidu.com/s/1HoUSZt0pAfeF4vPTWPngnQ 
提取码：yn3t


## For LgSVL Simulation 

### Configure on Linux

***Open Web-bridge***
```bash
roslaunch lgsvl sensor.launch
```
This launch file can open the web bridge between windows/linux, then you can subscribe topics from lgsvl on windows side.


### Install & Configure LgSVL on Windows

***Install LgSVL***
* [Download and Install LgSVL](https://www.lgsvlsimulator.com/docs/getting-started/#downloading-and-starting-simulator)
* [Configure the scenary](https://www.lgsvlsimulator.com/docs/build-instructions/)

***Configure LgSVL***
* [Configure Car parameters](https://www.lgsvlsimulator.com/docs/vehicles-tab/)
You need to configure the sensor outputs from for a given vehicle.

* [Configure simulator](https://www.lgsvlsimulator.com/docs/simulations-tab/)

* [Configure scene](https://www.lgsvlsimulator.com/docs/build-instructions/)
You can modify the scene and add custom video outputs for lgsvl.

## EVO Analysis


### Loop Closure Detection Updating


## TODO Lists


- [ ] Add LgSVL simulator
- [ ] Modify LiDAR msg from lgsvl
- [ ] Add loop closure detection
- [ ] Update Map optimization

