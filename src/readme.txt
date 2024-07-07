传感器插入顺序： 1.407串口    2.雷达    3.imu
记得赋权限
赋权限后直接运行roslaunch agvcar gr_nav.launch即可
发布点位可以参照：
rostopic pub /order std_msgs/Int32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0,1,2,3,4,5,6]" 

坐标系的三轴方向与earth一样。
激光雷达线指向x轴负向。
rqt可以查看tf树
保存地图：
rosrun map_server map_saver -f map_name