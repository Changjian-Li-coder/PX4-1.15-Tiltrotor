V2.1.0  --- 2026年4月19日发布

更改如下：
1.更改了控制分配代码allocation_calculation.cpp/hpp
2.删减了不必要的参数
3.（重要）调整坐标系：由前右下改为前左上，控制分配矩阵A进行调整
4.QGC参数如下：
   MY_SER_DEL_MAX = 100
   MY_SER_DEL_MIN = -100
   MY_THRUST_MAX = 10(重要）
   MY_SER_LP_A = 1
   MY_WRITE_TIME = 4
   其余保持默认参数
