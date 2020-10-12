# sensor_fusion_threading
Real-time sensor fusion for localization with threading when your company doesn't allow ROS.  

## candidates for EKF
1. implement from scratch(python) -> threading(python)
2. use filterpy ekf(python) -> threading(python)
3. implement from scratch(C++) -> threading(C++) -> cdll, cpython -> get it from python
4. use ekf from carND(C++) -> threading(C++) -> cdll, cpython -> get it from python
