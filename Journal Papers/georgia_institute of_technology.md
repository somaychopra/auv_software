# AUV (GIT) ([Link](https://robonation.org/sites/default/files/GeorgiaTech_2016_RoboSub_Journal.pdf))
## AAVS
* The Adept Autonomous Vehicle Simulation (AAVS) is a simulation environment developed inhouse at ASDL.It is coded in the C# programming language nd runs on the Microsoft Windows operating system.
## Hardware/Software interface
* A simple lightweight communication protocol was implemented for the Arduino to send to and receive data from the computer. Messages are separatedby a zero byte. Messages consist of one identification byte between 1 and 254 that states the type of message  (motor  command, pressure sensor value...), if the message can contain a zero (e.g. when it is a float) it is coded using the Consistent Overhead Byte Stuffing (COBS) to remove zeros. Hence the only zeros that appear are the ones that separate two messages.It proved to be easy to implement,efficient and reliable.
## Sensors simulation
* The simulated sensordata isused as input to the control and autonomy algorithms that will then be used on the actual vehicle in the hardware in the loop mode.AAVS can simulatesensor input to test the algorithm in the lab
* The  virtual  environment used  in  simulation  is  rendered using the OpenGL API. 3D models of the environment and the objects encountered in the competition pool aredesigned using  CAD  tools.
## Dynamics modeling
* The   dynamics   ofan   underwater   vehicle   have   been extensively described and modeled by T. Fossen[2], and this model  is  widely  used  for  the  modeling  and  control  of  both surface and underwater vehicles.
## Control
* The  control  problem  has  been  divided  in  two  parts.  An inner control loop stabilizes the system around the required angle  and  depth  in  the  world  coordinates.  This  inner  loop consists of the IMU and depth sensor, a state estimator (such as  an  extended  Kalman  filter)  and  a  Proportional  Integral Derivative (PID) controller.
* Four PIDs were implemented, one for each of the position on which there is direct measurements: pitch, roll, yaw, and depth. 
* The  outer  loop  uses  image-based  navigation.  It  extracts known  features  from  the  camera  images  to  set  the  desired position  and  angle. 
![](https://screenshotscdn.firefoxusercontent.com/images/b5a287d6-737e-4e13-97ea-13a248ba26e0.png/Screenshot_2019-05-24_%EF%80%A0_-_GeorgiaTech_2016_RoboSub_Journal_pdf.png)
## Machine vision
* The  Machine  Vision  algorithms  developed  to perform object recognition and tracking build upon the  widely  used and open-source OpenCVsoftware. Wrappers for thisnative C++library are available for most programming languages.
### Image Processing
An initial image processing step is performedto obtain a single-channel binary image on which blob recognition can then be performed.First, the color space is changed from Red Green  Blue  (RGB) toHue  Saturation  Value  (HSV).  Then, the  value  channel  is  extracted.Random   noise   is   removed using morphological    transformations.Finally,    an    adaptive thresholding is performed to obtain a binary image.
### Blob identification
Contour detection is used in order to detect blobs, or regions of interest that can then be further characterized.
### Objects identification
The  starting  gate  was  characterized  as  an object presenting two clusters of vertical lines(the two sides of the  gate) and a  single  cluster of horizontal lines (the  top bar of the gate). From the intersections of the horizontal and vertical  lines and  the  bottom  of the  vertical lines,  it is then possible to extract the position of the four corners of the gate and  define  a quadrilateral shapethrough  which  the  vehicle should navigate
### Object tracking
The particle  population  is  propagated  from  one  time  step  to  the next   by   adding   a   random   noise   to   allow   for   a   slight movement of the tracked object. Then, particles are affected a weight based on their distance to theresult from the object recognition.  Finally,  the  particles  are  resampled  and  the weights  are  reset.  The  particle  filter  approach  effectively results in a smoother and more reliable object-tracking.
