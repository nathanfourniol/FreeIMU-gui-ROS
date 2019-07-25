# FreeIMU calibration GUI for ROS

*Initial release (for arduino) :* http://www.varesano.net/blog/fabio/freeimu-magnetometer-and-accelerometer-calibration-gui-alpha-version-out

GUI adapt to connect ROS topic instead of arduino serial

Topics messages need to be :

* *sensor_msgs/Imu()* 
* *sensor_msgs/MagneticField*


**Dependencies :** PyQt4, pyqtgraph, PyOpenGL, numpy, scipy

**To start the gui :**
```python2 cal_gui.py```

See https://github.com/mjs513/FreeIMU-Updates/wiki/04.-FreeIMU-Calibration if you want more informations of how to use the gui. 
