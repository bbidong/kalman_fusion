# 卡尔曼实现传感器融合

------



### 目录结构

```
代码目录结构如下所示：
|___kalman_fusion
    |___bin 代码编译后的可执行文件及融合结果存放目录
    |___data 传感器融合的输入数据
    |        |___rm_gt.txt  真实轨迹
    |        |___measure.txt  传感器数据
    |        |___make_data.py  根据真实轨迹加噪声生成传感器数据
    |___src 传感器融合的源代码
    |      |___Eigen  第三方包
    |      |___algorithims
    |      |              |___kalmanfilter.cpp  卡尔曼算法的5个公式
    |      |              |___sensorfusion.cpp  初始化卡尔曼的参数矩阵
    |      |___interface
    |      |              |___measurement_package.h  定义一个类存储传感器数据
    |      |___main.cpp  主函数
    |___build.sh 编译脚本
    |___CMakeLists.txt 组建工程的CMake文件
    |___LICENSE Udacity的License
    |___Reame.md 使用说明
```

### 使用方法

系统
Linux Ubuntu 18.04

编译
$ mkdir build
$ bash build.sh

运行
$ ./bin/SensorFusion


### 链接

https://zhuanlan.zhihu.com/c_147309339
无人驾驶技术入门（十九）| 手把手教你实现多传感器融合技术
