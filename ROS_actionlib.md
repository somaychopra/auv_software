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
* __Simple Server__
  * Example :
    ```C++
      #include <ros/ros.h>
      #include <actionlib/server/simple_action_server.h>
      #include <actionlib_tutorials/FibonacciAction.h>

      class FibonacciAction
      {
      protected:

        ros::NodeHandle nh_;  //creating a nodehandle is necessary before declaring a node
        actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; // NodeHandle instance must be created       before this line. Otherwise strange error occurs.
        std::string action_name_;
        // create messages that are used to published feedback/result
        actionlib_tutorials::FibonacciFeedback feedback_;
        actionlib_tutorials::FibonacciResult result_;

      public:

        FibonacciAction(std::string name) :
          as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
          action_name_(name)
        {
        as_.start();
        }

        ~FibonacciAction(void)
        {
        }

        void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
        {
          // helper variables
          ros::Rate r(1);     //Rate at which to send the goal to the server
          bool success = true;

          // push_back the seeds for the fibonacci sequence
          feedback_.sequence.clear();
          feedback_.sequence.push_back(0); // pushback is a function to add elements to a vector
          feedback_.sequence.push_back(1);

          // publish info to the console for the user
          ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

          // start executing the action
          for(int i=1; i<=goal->order; i++)
          {
            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
              ROS_INFO("%s: Preempted", action_name_.c_str());
              // set the action state to preempted
              as_.setPreempted();
              success = false;
              break;
            }
            feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
            // publish the feedback
            as_.publishFeedback(feedback_); 
            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep();  //to let the next requests wait until the current is processing 
          }

          if(success)
          {
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
          }
        }


      };


      int main(int argc, char** argv)
      {
        ros::init(argc, argv, "fibonacci");  //"fibonacci" will be the name of the node

        FibonacciAction fibonacci("fibonacci");
        ros::spin();

        return 0;
      }
      ```
   * Final CMakeLists.txt
      ```
      cmake_minimum_required(VERSION 2.8.3)
      project(actionlib_tutorials)

      find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs)
      find_package(Boost REQUIRED COMPONENTS system)

      add_action_files(
        DIRECTORY action
         FILES Fibonacci.action
      )

        generate_messages(
        DEPENDENCIES actionlib_msgs std_msgs
        )

        catkin_package(
           CATKIN_DEPENDS actionlib_msgs
        )

        include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

        add_executable(fibonacci_server src/fibonacci_server.cpp) 

        target_link_libraries(
           fibonacci_server
          ${catkin_LIBRARIES}
        )

        add_dependencies(
           fibonacci_server
          ${actionlib_tutorials_EXPORTED_TARGETS}
        )
        ```
  * Running the action server 
    ```
    $ roscore
    ```
    ```
    $ rosrun actionlib_tutorials fibonacci_server
    ```
    Then to check whether the action is running properly list the topics being published
    ```
    $ rostopic list -v
    ```
    Alternatively
    ```
    $ rqt_graph
    ```
* __Simple client__
  *
