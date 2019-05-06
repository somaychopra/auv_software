# ROS actionlib

* The actionlib package helps to get feedback from the server even while the request is still processing , unlike ros services in     which we just have to wait for a single response until the task gets completed.
* The ActionClient and ActionServer communicate via a ROS Action Protocol.
  ![Server client interaction](http://wiki.ros.org/actionlib?action=AttachFile&do=get&target=client_server_interaction.png)
* __.action File__
  * goal : a goal that can be sent to an ActionServer by an ActionClient. It  contains information about where the robot should move to in the world. 
  * feedback : Feedback provides server implementers a way to tell an ActionClient about the incremental progress of a goal.
  * result : A result is sent from the ActionServer to the ActionClient upon completion of the goal. This is different than feedback, since it is sent exactly once.
  * Example of an action file(to be created in a new action folder inside the src folder of the package.) <br />
    ```
    #goal definition
     int32 order
      ---
    #result definition
    int32[] sequence
    ---
    #feedback
    int32[] sequence
      ```
   * Changes to be made in the CMakeLists.txt <br />
     add the actionlib_msgs package to the find_package macro's argument:
     ```
     find_package(catkin REQUIRED COMPONENTS actionlib_msgs)
     ```
     use the add_action_files macro to declare the actions you want to be generated: 
     ```
     add_action_files(
     DIRECTORY action
     FILES Fibonacci.action
     )
     ```
     call the generate_messages macro, not forgetting the dependencies on actionlib_msgs and other message packages like        std_msgs:
     ```
     generate_messages(
     DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
     )
     ```
     add actionlib_msgs to catkin_package macro like this: 
     ```
     catkin_package(
     CATKIN_DEPENDS actionlib_msgs
     )
     ```

   * Changes to be made in the package.xml : <br />
      add -  <exec_depend>message_generation</exec_depend>
   
   * Now by following, automatically generate msg files of your action files, and also see the result. 
    ```
    $ catkin_make
    $ ls devel/share/actionlib_tutorials/msg/
    ```
