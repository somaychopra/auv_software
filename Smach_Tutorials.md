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
### Passing and using user data
The input and output data of a state is called userdata of the state. <br />
__Passing user data__
```python
 class Foo(smach.State):
     def __init__(self, outcomes=['outcome1', 'outcome2'],
                        input_keys=['foo_input'],
                        output_keys=['foo_output'])

     def execute(self, userdata):
        # Do something with userdata
        if userdata.foo_input == 1:
            return 'outcome1'
        else:
            userdata.foo_output = 3
            return 'outcome2'

```
* The input_keys list enumerates all the inputs that a state needs to run. A state declares that it expect these fields to exist in the userdata. The execute method is provided a copy of the userdata struct. The state can read from all userdata fields that it enumerates in the input_keys list, but it can't write to any of these fields. 
* The output_keys list enumerates all the outputs that a state provides. The state can write to all fields in the userdata struct that are enumerated in the output_keys list. <br />
__Connecting user data__
```python
sm_top = smach.StateMachine(outcomes=['outcome4','outcome5'],
                          input_keys=['sm_input'],
                          output_keys=['sm_output'])
  with sm_top:
     smach.StateMachine.add('FOO', Foo(),
                            transitions={'outcome1':'BAR',
                                         'outcome2':'outcome4'},
                            remapping={'foo_input':'sm_input',
                                       'foo_output':'sm_data'})
     smach.StateMachine.add('BAR', Bar(),
                            transitions={'outcome2':'FOO'},
                            remapping={'bar_input':'sm_data',
                                       'bar_output1':'sm_output'})

```
The remapping field maps the in/output_key of a state to a userdata field of the state machine. So when you remap 'x':'y':

* x needs to be an input_key or an output_key of the state, and 
* y will automatically become part of the userdata of the state machine. 
> Note that remapping is not required when the user data names used in your state are the same as the user data names used by the state machine. However, remapping makes the connections very explicit, so it is recommended to always specify remapping, even something like "remapping={'a':'a'}".<br />

![](http://wiki.ros.org/smach/Tutorials/User%20Data?action=AttachFile&do=get&target=user_data.png)
__Example:__[__Link__](http://wiki.ros.org/smach/Tutorials/User%20Data%20Passing)


