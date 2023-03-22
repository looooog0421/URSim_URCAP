## UR5e 使用说明
修改`all.launch`中机器人ip, 如果是仿真器, 则ip为: 0.0.0.0

1. 运行驱动

    roslaunch ur5e_basic all.launch

2. 仿真器或者示教器内更改urcap的ip并运行
3. 运行测试文件

        rosrun ur5e_basic ur_test     

## 文件夹内容
- data: 实验数据存放目录,首先通过`getSourceCodePath()`获取代码路径,然后增加`/data/`即可
- matlab: m函数,用于绘制实验数据的图像
