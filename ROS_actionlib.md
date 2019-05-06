# ROS actionlib

* The actionlib package helps to get feedback from the server even while the request is still processing , unlike ros services in     which we just have to wait for a single response until the task gets completed.
* The ActionClient and ActionServer communicate via a ROS Action Protocol.
  ![Server client interaction](http://wiki.ros.org/actionlib?action=AttachFile&do=get&target=client_server_interaction.png)
* __.action File__
  * goal : a goal that can be sent to an ActionServer by an ActionClient. It  contains information about where the robot should move to in the world. 
  * feedback : Feedback provides server implementers a way to tell an ActionClient about the incremental progress of a goal.
  * result : A result is sent from the ActionServer to the ActionClient upon completion of the goal. This is different than feedback, since it is sent exactly once.
