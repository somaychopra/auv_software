# Matsya 4A ([Link](https://robonation.org/sites/default/files/IITBombay_2016_RoboSub_Journal.pdf))
* The  Software  Stack  of  Matsya  4A  has  beendeveloped on top of Robot Operating System(ROS), developed at Willow Garage
* Off-Board Computing:  This layer consistsof specialized hardware like filters, ampli-fiers, ADC, GPIO, etc.  
* Firmware/Driver  Layer:   Responsible  forall of the Hardware Abstraction, this layerhelps abstract data from the sensors andpresent them as processes to the SBC
* Middle layer:  This layer is responsible forprocessing information from sensors suchas IMU, DVL and Cameras and presentingit to the higher level logic nodes.
* Top  layer:   This  layer  uses  informationfrom  the  middle  layer  to  generate  actua-tion  commands.   It  also  provides  a  real-time interface to monitor Matsya ́s perfor-mance and as well as to implement indi-vidual missions.
## Localization
*  It  is  responsible  for  filtering  and  fusing  datafrom  inertial  and  acoustic  sensors  to  provide the top layer with a single source of all relevant localization data. 
## Inter board communication
* The   communication   between   SBC   and   onboard controller is handled over TCP/IP andUDP protocol which provide a reliable mecha-nism for transferring of data. 
## Navigation
Based  on  a  state  machine  architecture,  itchooses the motion to be performed based onchecks on the localization data with respect tothe desired set point.
## Mission planner
* Transition State:  When the currently ac-tive  task  is  not  in  the  range  of  Matsya's sensors, Matsya navigates to the optimumpoint using a user-fed belief map.  As soonas Matsya has the task within range of itssensors, it switches to the Scan State.
* Scan  State:   When  the  currently  activetask  is  in  the  range  of  Matsya ́s  sensors,Matsya tries to search around itself usinga combination of its history and user-fedscan  plans.
* Execution State:  When Matsya enters theExecution State from Scan State, there isa very high probability that the task canbe completed.
* On losing trackof the task, it switches to scan state so asto position itself properly again.
## Interfaces
### Debug iterface
The underlyingidea  behind  developing  Matsya  4A’s  De-bug Interface was to provide the user max-imum  control  over  Matsya  and  to  allowtask  execution  with  fewer  button  clicks,as  well  as  easier  modification  of  parame-ters to be viewed or controlled.  This de-manded for a responsive and modular UIwhich was met using the rqt plugins.
### Vision Interface
This interface is used to set various visionrelated parameters which are to be tuneddepending  on  the  environmental  condi-tions.
### Map interface
Provides  a  drag  and  drop  interface  toschedule a list of tasks to perform on anestimated map of the arena.
## Vision
Uses Intel’s OpenCV 2.4 library.
* Framework:  The image processing code isimplemented  as  a  library  employing  ob-ject oriented philosophy for different pro-cessing  techniques. 
* Camera  control  and  pre-processing:   Toaccount   for   the   varying   environmental conditions such as illumination variation,brightness   artifacts,   sunlight   reflection,etc., auto exposure and gain control algo-rithms have been implemented which up-date the camera parameters in real-time.
* Processing  techniques:   The  major  pro-cessing  modules  include  color  detection,shape  based  validation,  region  growing,contour   analysis   and   machine   learningbased  object  detection  and  classification.
* Parameter tuning: Some of the image pro-cessing techniques require some tuning of parameters for better performance.
## Controls
* Precise control forfive  degrees  of  freedom,  namely,  surge,  sway,heave,  pitch  and  yaw.
* The  control  loop  runsat a frequency of 10 Hz, acquiring localizationdata  from  the  DVL  and  IMU  and  generatingappropriate  PWM  signals  for  each  of  the  sixthrusters. 
* The kinematic  and  dynamic  controller  pair  is  able  to achieve  global  position  and  velocity  reference tracking.
