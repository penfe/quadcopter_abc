## 简介

爱好四轴飞行器，还想了解其飞行的数学模型。编写一个简单的仿真程序，可能是最完整和最经济的学习手段。
1. 首先，**根据牛顿-欧拉方程建立四旋翼飞行器的动力学和运动学模型**，从而可以根据飞行器的实时状态（受力、位姿、线速度和角速度等数据），计算输出其实时位姿变化（线加速度和角加速度），并可视化仿真程序。
2. 其次，进行线性模型化简，**根据 PID 控制律设计底层飞行控制器**，以飞行器的实时状态和飞行指令（定点等）为输入，实时计算对应的螺旋桨转速数据，并在仿真程序中进行测试和验证。

<img src="demo.gif" alt="仿真示意图" title="示意图来自知乎抱歉忘了原作者是谁">

参考资料
* [多旋翼飞行器设计与控制](https://study.163.com/course/courseMain.htm?courseId=1003715005)
* [玖辞丶四旋翼飞行器建模](https://zhuanlan.zhihu.com/p/349306054)

详见 [技术细节笔记](./四旋翼飞行器仿真.ipynb)