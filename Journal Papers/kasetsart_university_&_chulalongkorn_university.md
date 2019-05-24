# Zeabus AUV
## Mission planner group module
* Sensors such as IMU, cameras, and hydrophones, will send data through communication  channels,  and  then  the  sensor  fusion software  module  will  fuse  all  data  together  in  order  to process in the next step
![](https://screenshotscdn.firefoxusercontent.com/images/ec2539af-5334-4826-8d8f-f66ed121c82c.png/Screenshot_2019-05-24_Microsoft_Word_-_2016_team_description_paper_docx_-_KasetsartUniandChulalongkornUni_2016_RoboSub_Jour%5B...%5D.png)
## Control system module
* Orientation of the robot can be estimated by fusing all measured data using Unscented Kalman Filter (UKF) [2]. Angle estimates are available as Euler angle or quaternion angle outputs.
![](https://screenshotscdn.firefoxusercontent.com/images/cc57bf92-6e95-42b9-ac07-d6b3869276c9.png/Screenshot_2019-05-24_Microsoft_Word_-_2016_team_description_paper_docx_-_KasetsartUniandChulalongkornUni_2016_RoboSub_Jour%5B...%5D.png)

## Vision system module
* Written using OpenCV and TensorFlow in python and c++.
![](https://screenshotscdn.firefoxusercontent.com/images/51f5fbb1-57a1-4ec8-bcf4-fca5e980c63b.png/Screenshot_2019-05-24_Microsoft_Word_-_2016_team_description_paper_docx_-_KasetsartUniandChulalongkornUni_2016_RoboSub_Jour%5B...%5D.png)
* In the CNN (Convolution Neural Network) module, the image kernel is generated from training data and used to detect and classify which part of the input image from the camera are informative. Then, the image will be segmented for  further  processing.  The  segmented  images  are  then passed to Shape detection module to do further processing about the location of that object.  
* The foreign object detection module will initially detect if there are any objects that should not be in water, on the floor,  or  in  the  background.
* To  perform  foreign  object detection, the captured image is analyzed with the hysteresis thresholding.
* Average colors of the image are computed so that the color of the “background” can be extracted. This color is then expanded into a range in a color space, which are YUV or YCbCr. 
* The color detection module uses colors that are indexed in HSV (Hue-Saturation-Value) color space because HSV is easy to represent an index of colors by hue  values.  After images are acquired from AUV cameras, the imaging data are converted to HSV
## Hydrophone processing module
software methodology:
* Sampling : Analogsignals converted to digital.
* Pulse detection and demodulation : Detect pinger pulses.
* Delay time estimation : Accurately extract one pulse of pinger signals from all 4 hydrophones for further processing. 
* Bearing estimation : The azimuth angle and the elevation angle are computed in  this  step  as  the  output  of  the  system.  Particle  filter algorithm [6] is used to reduce noises in this step. Particle filter  is  a  general  Monte  Carlo  (sampling)  method  for performing inference in state-space models where the state of a system evolves in time and information about the state is obtained via noisy measurements made at each time step.
