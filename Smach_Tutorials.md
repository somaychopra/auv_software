# Smach Tutoials
Smach, which stands for "State Machine", is a powerful and scalable Python-based library for hierarchical state machines. The Smach library does not depend on ROS, and can be used in any Python project. The executive_smach stack however provides very nice integration with ROS, including smooth actionlib integration and a powerful Smach viewer to visualize and introspect state machines.
### Creating a State Machine
```python
class Foo(smach.State):
     def __init__(self, outcomes=['outcome1', 'outcome2']):
       # Your state initialization goes here

     def execute(self, userdata):
        # Your state execution goes here
        if xxxx:
            return 'outcome1'
        else:
            return 'outcome2'

```
* In the init method you initialize your state class.
* In the execute method of a state the actual work is done.
* When a state finishes, it returns an outcome. Each state has a number of possible outcomes associated with it. An outcome is a user-defined string that describes how a state finishes. A set of possible outcomes could for example be ['succeeded', 'failed', 'awesome']. The transition to the next state will be specified based on the outcome of the previous state. 
### Adding states to a state machine
```python
sm = smach.StateMachine(outcomes=['outcome4','outcome5'])
  with sm:
     smach.StateMachine.add('FOO', Foo(),
                            transitions={'outcome1':'BAR',
                                         'outcome2':'outcome4'})
     smach.StateMachine.add('BAR', Bar(),
                            transitions={'outcome2':'FOO'})

```
![](http://wiki.ros.org/smach/Tutorials/Getting%20Started?action=AttachFile&do=get&target=simple.png)<br />
__Example of a simple state machine__: [__Link__](http://wiki.ros.org/smach/Tutorials/Simple%20State%20Machine)<br />
Running the example
```
$ git clone https://github.com/eacousineau/executive_smach_tutorials.git
$ cd executive_smach_tutorials
$ ./examples/state_machine_simple.py
```
```
$ roscd smach_tutorials
$ ./examples/state_machine_simple.py
```

