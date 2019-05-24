# Design Rationale ([Link](https://robonation.org/sites/default/files/KennesawStateUni_2016_RoboSub_Journal.pdf))
* Used ROS framework to easily communicate between nodes.
![](https://screenshotscdn.firefoxusercontent.com/images/2332568c-bc9e-4c08-9b55-9419f2b2f749.png/Screenshot_2019-05-24_Microsoft_Word_-_RoboSub_JournalTemplate_v2_1_docx_-_KennesawStateUni_2016_RoboSub_Journal_pdf.png)
*  It listens to the outputs of the ZED, a camera node which translates image into a color laser chart.
* The PixHawk(Figure 7),a motor autopilot with a built in compass and gyroscope, controls 6 motors as well as telemetry. From that point, our node then sends an initial inquiry to the PixHawk to identify the current location of the AUV.
* C++ used to write all the codes of the nodes.
* Runs on Ubuntu 14.04.
