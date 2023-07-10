
# ros坐标系

主要是两个坐标系，map坐标系和vehicle坐标系.
一般而言，map 坐标系是全局坐标系，定位坐标系也是基于map坐标系。
vehicle 坐标系是局部坐标系，大部分的数据都是在vehicle坐标系中计算的。

* rviz显示图像的时候，如何让视角在车辆坐标系上？
在rviz中选择target tf即可。

* 在画各种图形的时候，如何各个图形确定属于那一个坐标系。
  每一个图形会记录属于哪一个tf。

Pose 和 TransformStamped是可以相互转换的。

# ros参数

# ros launch

# 节点

# action

# 消息