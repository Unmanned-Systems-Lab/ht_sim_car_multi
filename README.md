# 安装

- 安装依赖包

  ```
  sudo apt-get install ros-$(ros_version)-ros-control ros-$(ros_version)-ros-controllers
  ```

- 下载编译

  ```
  mkdir myh_ws
  cd myh_ws
  git clone https://gitee.com/ht-hlf/ht_sim_car_multi.git src
  catkin build
  source devel setup.bash
  ```

# 仿真

## 单车 仿真

- ```
  roslaunch hunter_se_gazebo hunter_se_empty_world_sensor.launch 
  ```

**演示效果：**

![](sim_myh.assets/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202023-01-14%20131501-16736733311963.png)

## 多车 无传感器仿真

> 有里程计话题，但是没有固态激光雷达 gps等传感器发布

- ```
  roslaunch sim_myh new_sim_car_sum_5.launch
  ```

**演示效果：**

![](sim_myh.assets/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202023-01-14%20125933-16736724115222.png)



## 多车 有传感器仿真

- ```
  roslaunch sim_myh new_sim_car_sum_5_sensor.launch
  ```

**演示效果：**

![](sim_myh.assets/%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE%202023-01-14%20125154-16736720366621.png)