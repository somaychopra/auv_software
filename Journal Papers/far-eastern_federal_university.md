# FEFU RoboSub 2016 ([Link](https://robonation.org/sites/default/files/FarEasternFedUni_2016_RoboSub_Journal.pdf))
## ROS structure
* Ros is used as an interprocess communication  frame-work .
* Messages are divided into two groups commands and data messages.
* Only one module csn send the data message of a specific type but everyone may subscribe to see it. 
![](https://screenshotscdn.firefoxusercontent.com/images/798fefdf-0029-4f78-8259-9f5de2fd7ee3.png/Screenshot_2019-05-23_FarEasternFedUni_2016_RoboSub_Journal_pdf.png)
## Web GUI
* Created web interfaced console to visualize numerical data for messages in real time.
* It uses roswww package as a server running on a vehicle computerand roslib.js to implement client side part.
* plot the numerical values overtime an  send commands to the vehicle through a web page which may be accessed through every device even not having ROS on it. 
## Navigation node description
* Filtration and broadcasting data from IMU,DVL and other sensors.
* Path's calculation based on informaton of velocity and heading.
## Mission node description
* The mission node contains the motion and actions logic of the vehicle.
* This node consists of separate tasks, which are executed in a linear sequence.The order of execution is described in a set of configuration files.There is a global map that stores information about recently found orange stripes. This information is used for vehicle positioning before starting the next scheduled task.
* YAML is used for mission configuration files.
## Motion controls
* Motion control divided in two functional part:motion node and TCU (thruster control unit or propulsion system) node.
* Motion solve high level motion problem including stabilization. The vehicle motion control is performed in the event loop. The data from navigation module is processed at the each iteration of this loop.
* Separate regulator implemented as module is created for each control command.
* Such a module subscribes to a message, that describes the module command and activates the regulator.
* PID controller is used on this vehicle for solving mathematical control problem.
* Propulsion system node is to receive thrusts and torques from motion module and calculates control signals for each thruster on the basis of received values.
## Video module
* OpenCV library is used for image processing. 
* Preprocessing:It’s done by applying filters or clustering algorithms to correct underwater colors, remove noise and small useless details. Color correction is done by increasing red channel in RGB image. Then three ways of image preprocessing are used: median blur, Gaussian blur. Median and Gaussian filters are implemented in OpenCV
* Color binarization:The source image is translated to HSV color space and the binarization is executed by hue and saturation threshold.
* Contour analysis : It is based on OpenCV impementations of Suzuki-Abe algorithm and polygon approximation,a custom Hough implementation is also used.
## Implementation
* C++ is used for software implementation.
* Boost libraries are used.
* Cmake is to build the system.
* Bash scripts are used for launching.
* Git version control system and central private repository on bitbucket.org are used to control team development process.
