# YellowFin - ([Link](https://robonation.org/sites/default/files/EmbryRiddle_2016_RoboSub_Journal.pdf))
* The  main  software  packages usedon  the subs  are  as  follows:  Python  2.7,  NumPy, OpenCV,   and   matplotlib,   as   well   as   the Arduino  IDE
* NumPy is used for  array  and  matrix  manipulation.  OpenCV is used for vision processing and part of our GUI.  Matplotlib  is  used  to  graph all  of  the data thesub  records.
* Operating system used is Windows 7.
![](https://screenshotscdn.firefoxusercontent.com/images/a7fe1978-dca9-460e-8272-312288aff7d7.png/Screenshot_2019-05-22_Microsoft_Word_-_EmbryRiddle_2016_RoboSub_Journal_docx_-_EmbryRiddle_2016_RoboSub_Journal_pdf.png)
## Software Design
### Hardware Communication
* The   motor   controller   board   is   interfaced with  through  a  UDP  connection.  Through actuator.py,  each  of  the  six  motors  can  be independently   controlled   through   i2c.
* The  motors.py  script  handles  higherlevel movement  commands,  translating  them  into individual motor movements. 
* The  motor  voltage  and  current  along  with the   barometric   pressure   and   temperature sensors    are    read    using    controlpcb.py, interfacing   with   the   Galileo.
* Compass.py interfaces directly with the VN-100   IMU   via   serial   RS232.   Images   are retrieved from the camera using OpenCV. 
* The  daq  is  communicated  with  through  a LabView  program  compiled  into  a  dll.  This dll  connects  to  the  daq  through  ethernet protocols.  Through  hydrophones.py,  this  dll is  loaded  and  the  angles  computed  by  each hydrophone pair are returned.
### Data Processing
*  Through  a  set  of conditionalstatements    and    the    vision processing  scripts  (gateDetect,  pathDetect, buoyDetect,    maneuverDetect)    the    state manager  determines  what  actions  should  be sent  to  motors.py.  The  vision  processing scripts  interpret  a  target  heading  and  depth, and  the  state  manger  communicates  this  to the   motor   control   script   which   uses   the current  heading  to  determine  motor  power changes.
### Data logging
*  Writing  to  any  of  the  logs  is handled  through  logger.py. 
*  On  startup,  both logs  are  automatically  created.  The  data  is written   directly   to   text   files,   instead   of through  a  buffer,  to  ensure  that  if  a  crash  is to occur, the logs are intact.
* The   sensor   logs   function   much   like   the action  logs.  They  are  comma  delimited  text files that are named with the current time in ERAUAUV2016Journal Paper9|Pageseconds.  On  average,  the  data  from  every sensor  is  logged  every  half  second.  These logs     have     been     crucial     for     finding relationships between some of our data such as internal case temperature and pressure.
### Value manipulation
* To  facilitate expedient value tuning, the constants used in the   motors   for   calibration,   heading   and depth  control  PI  controllers,  thresholds  in vision and more can be altered by loading a text file using constantEditor.py.
## Software Algorithm
* Load   waypoints   folder   containing each  point  of  interest,  and  the  angle between  them.  This  information  is pre entered based off the condition of the pool. 
* Using file as a reference, head in the direction of the next waypoint, while using  thecameras  to  attempt  to  lock onto  the  object.  If  the  target  can  be seen,  head  towards  it.  If  not  hold heading  and  continue  forwards.  A simple   PI   controller   is   used   to maintain heading. Additionally,depth  is  also  monitored  though  each control  cycle.  If  the measured  depthis   out   of   the   accepted   range, a correction    is    made    using    a    P-controller. 
* If  waypoint  is located,run  the  script associated  with  that  task,  update  the waypoint  location,  and  return  to  the main control loop.
## Vision Processing
*  By  having  an image  with  few  color  defects,  a  threshold can  easily  be  applied.To  increase the  color accuracy  of  the  image,  the  OpenCV  is  used to  normalize  the  histogram.This  ensures that  the  lighting  conditions  between  images are  consistant.
* A  binary  threshold is  thenapplied  to  the  Hue,  Saturation,  and  Value (HSV)  planes  of  the  image. Once  a  binary threshold is applied,contours are used to get an  array  of  onscreen  objects.
* Once  a  list  of objects  is  made,they  are  sorted  by  area, location,    and    shape.
