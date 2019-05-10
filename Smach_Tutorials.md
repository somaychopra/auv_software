# Smach Tutorials
Smach, which stands for "State Machine", is a powerful and scalable Python-based library for hierarchical state machines. The Smach library does not depend on ROS, and can be used in any Python project. The executive_smach stack however provides very nice integration with ROS, including smooth actionlib integration and a powerful Smach viewer to visualize and introspect state machines.
## Learning SMACH
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

![](http://wiki.ros.org/smach/Tutorials/User%20Data?action=AttachFile&do=get&target=user_data.png)<br />
__Example Code__ __:__ [__Link__](http://wiki.ros.org/smach/Tutorials/User%20Data%20Passing)
### Nesting state machines
* For this example, we create a number of states, each with a number of outcomes, input keys and output keys specified.
```python
# State Foo
  class Foo(smach.State):
     def __init__(self, outcomes=['outcome1', 'outcome2'])
     
     def execute(self, userdata):
        return 'outcome1'


  # State Bar
  class Bar(smach.State):
     def __init__(self, outcomes=['outcome1'])
     
     def execute(self, userdata):
        return 'outcome4'


  # State Bas
  class Bas(smach.State):
     def __init__(self, outcomes=['outcome3'])
     
     def execute(self, userdata):
        return 'outcome3'


```
* We create a top level state machine, and start adding states to it. One of the states we add is another state machine.
```python
 # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])

    # Open the container
    with sm_top:

        smach.StateMachine.add('BAS', Bas(),
                               transitions={'outcome3':'SUB'})

        # Create the sub SMACH state machine 
        sm_sub = smach.StateMachine(outcomes=['outcome4'])

        # Open the container 
        with sm_sub:

            # Add states to the container 
            smach.StateMachine.add('FOO', Foo(),
                                   transitions={'outcome1':'BAR', 
                                                'outcome2':'outcome4'})
            smach.StateMachine.add('BAR', Bar(),
                                   transitions={'outcome1':'FOO'})

        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'outcome4':'outcome5'})

```
![](http://wiki.ros.org/smach/Tutorials/Create%20a%20hierarchical%20state%20machine?action=AttachFile&do=get&target=sm_expanded.png)

__Example Code__ __:__ [__Link__](http://wiki.ros.org/smach/Tutorials/Nesting%20State%20Machines)<br />
* The only point to take away from this is that every state machine is also a normal state. 
* So you can add a state machine to another state machine in the same way you add a state to a state machine.
### Calling actions from state machine
Import the lib first : 
```python
from smach_ros import SimpleActionState
```
* Empty goal message
```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    smach.StateMachine.add('TRIGGER_GRIPPER',
                           SimpleActionState('action_server_namespace',
                                             GripperAction),
                           transitions={'succeeded':'APPROACH_PLUG'})

```
* Fixed goal message
```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    gripper_goal = Pr2GripperCommandGoal()
    gripper_goal.command.position = 0.07
    gripper_goal.command.max_effort = 99999
    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        goal=gripper_goal),
                      transitions={'succeeded':'APPROACH_PLUG'})
```
* Goal from user data
```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        goal_slots=['max_effort', 
                                                    'position']),
                      transitions={'succeeded':'APPROACH_PLUG'},
                      remapping={'max_effort':'user_data_max',
                                 'position':'user_data_position'})
```
Imagine you have a number of fields in the user data that already contain all the structs you need for your goal message. Then you can simply connect the userdata directly to the fields in the goal message. So, from the example above we learn that the gripper action has two fields in its goal: max_effort and position. Imagine our userdata contains the corresponding fields user_data_max and user_data_position. The code below connects the corresponding fields. 
* Goal Callback
```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    def gripper_goal_cb(userdata, goal):
       gripper_goal = GripperGoal()
       gripper_goal.position.x = 2.0
       gripper_goal.max_effort = userdata.gripper_input
       return gripper_goal

    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        goal_cb=gripper_goal_cb,
                                        input_keys=['gripper_input'])
                      transitions={'succeeded':'APPROACH_PLUG'},
                      remapping={'gripper_input':'userdata_input'})

```
You can get a callback when the action needs a goal, and you can create your own goal message on demand. 
* Result to user data
```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        result_slots=['max_effort', 
                                                      'position']),
                      transitions={'succeeded':'APPROACH_PLUG'},
                      remapping={'max_effort':'user_data_max',
                                 'position':'user_data_position'})

```
* Result callback
The result callback is very similar to the goal callback. It allows you to read any data from the action result fields, and even return a different outcome than the default 'succeeded', 'preempted', 'aborted'.<br />
In the result callback you get the status of the action, which tells you if the action succeeded, aborted or was preempted. Plus you get access to the userdata, and the result of the action. <br />
Optionally you can return a different outcome from the result callback. If you don't return anything, the state will return with the corresponding outcome of the action. 


```python
sm = StateMachine(['succeeded','aborted','preempted'])
with sm:
    def gripper_result_cb(userdata, status, result):
       if status == GoalStatus.SUCCEEDED:
          userdata.gripper_output = result.num_iterations
          return 'my_outcome'

    StateMachine.add('TRIGGER_GRIPPER',
                      SimpleActionState('action_server_namespace',
                                        GripperAction,
                                        result_cb=gripper_result_cb,
                                        output_keys=['gripper_output'])
                      transitions={'succeeded':'APPROACH_PLUG'},
                      remapping={'gripper_output':'userdata_output'})
```
__Example Code__ __:__[Link](http://wiki.ros.org/smach/Tutorials/Simple%20Action%20State)
### Concurrent state machines
![](http://wiki.ros.org/smach/Tutorials/Concurrent%20States?action=AttachFile&do=get&target=concurrence2.png)<br />
__Example Code__ __:__[__Link__](http://wiki.ros.org/smach/Tutorials/Concurrent%20States)
* This code helps in understanding two states running in parallel.
* For example the states FOO and BAR running at the same time when FOO gives outcome2 and BAR gives outcome1 then the outcome of BAS comes out to be outcome5otherwise a default outcome4.

## SMACH Containers
### State machine container
* Creating a State Machine<br />
Import the state machine
```python
from smach import StateMachine
```
Since a SMACH StateMachine also provides a State interface, its outcomes and userdata interactions must be specified on construction. 
```python
sm = StateMachine(outcomes = ['outcome1', 'outcome2'],
                  input_keys = ['input1', 'input2'],
                  output_keys = ['output1', 'output2'])
```
Similarly to the SMACH State interface, input keys and output keys are optional. The constructor signature is: 
```python
__init__(self, outcomes, input_keys=[], output_keys=[]) 
```
* Adding States
When adding states to a state machine you first specify the state machine you want to add states to. This can be done by using Python's "with" statement. You can think of this like "opening" the container for construction. It creates a context in which all subsequent add* calls will apply to the open container. 
```python
with sm:
    StateMachine.add('FOO',
                     FooState(),
                     {'outcome2':'FOO',
                      'outcome3':'BAR'})
    StateMachine.add('BAR',
                     BarState(),
                     {'outcome3':'FOO',
                      'outcome4':'outcome2'})
```
The above example adds the two states labeled "FOO" and "BAR", of type "FooState" and "BarState", respectively. There are optional arguments to the static add method. The signature of the add method is: 
```python
add(label, state, transitions=None, remapping=None)
```
### Concurrence container
Importing
```python
from smach import Concurrence
```
* Concurrence Outcome Map
The outcome map of a SMACH concurrence specifies the policy for determining the outcome of the concurrence based on the outcomes of its children. Specifically, the map is a dictionary where the keys are potential outcomes of the concurrence, and the values are dictionaries mapping child labels onto child outcomes. 
```python
cc = Concurrence(outcomes = ['outcome1', 'outcome2'],
                 default_outcome = 'outcome1',
                 input_keys = ['sm_input'],
                 output_keys = ['sm_output'],
                 outcome_map = {'succeeded':{'FOO':'succeeded',
                                             'BAR':'outcome2'},
                                'outcome3':{'FOO':'outcome2'}})
with cc:
    Concurrence.add('FOO', Foo())
    Concurrence.add('BAR', Bar())
```
 * The example above specifies the following policy: <br />
     * When 'FOO' has outcome 'succeeded' and 'BAR' has outcome 'outcome2', the state machine will exit with outcome 'succeeded'. 
     * When 'FOO' has outcome 'outcome2', the state machine will exit with outcome 'outcome3', independent of the outcome of state BAR. 
     
* Callbacks
If you want full control over a concurrence state machine, you can use the callbacks it provides, the child_termination_cb and the outcome_cb: 
```python
# gets called when ANY child state terminates
def child_term_cb(outcome_map):

  # terminate all running states if FOO finished with outcome 'outcome3'
  if outcome_map['FOO'] == 'outcome3':
    return True

  # terminate all running states if BAR finished
  if outcome_map['BAR']:
    return True

  # in all other case, just keep running, don't terminate anything
  return False


# gets called when ALL child states are terminated
def out_cb(outcome_map):
   if outcome_map['FOO'] == 'succeeded':
      return 'outcome1'
   else:
      return 'outcome2'


# creating the concurrence state machine
sm = Concurrence(outcomes=['outcome1', 'outcome2'],
                 default_outcome='outcome1',
                 input_keys=['sm_input'],
                 output_keys=['sm_output'],
                 child_termination_cb = child_term_cb,
                 outcome_cb = out_cb)
                 
with sm:
   Concurrence.add('FOO', Foo(),
                   remapping={'foo_in':'input'})

   Concurrence.add('BAR', Bar(),
                   remapping={'bar_out':'bar_out'})
 
```
The child_termination_cb is called every time one of the child states terminates. In the callback function you can decide if the state machine should keep running (return False), or if it should preempt all remaining running states (return True).<br />
The outcome_cb is called once when the last child state terminates. This callback returns the outcome of the concurrence state machine. 
### Sequence Container
The Sequence container is a StateMachine container, extended with auto-generated transitions that create a sequence of states from the order in which said states are added to the container.
* Creating a sequence container
Import the sequence type
```pyhton
from smach import Sequence
```
A container Sequence has its outcomes, specified on construction, along with the 'connector_outcome' which is used for the automatic transitions. The constructor signature is: 
```python
__init__(self, outcomes, connector_outcome):
```
* Adding the states is same as we have seen before.
* Example:
```python
sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        connector_outcome = 'succeeded')
with sq:
    Sequence.add('MOVE_ARM_GRAB_PRE', MoveVerticalGripperPoseActionState())
    Sequence.add('MOVE_GRIPPER_OPEN', MoveGripperState(GRIPPER_MAX_WIDTH))
    Sequence.add('MOVE_ARM_GRAB',     MoveVerticalGripperPoseActionState())
    Sequence.add('MOVE_GRIPPER_CLOSE', MoveGripperState(grab_width))
    Sequence.add('MOVE_ARM_GRAB_POST', MoveVerticalGripperPoseActionState())
```
### Iterator Container
The iterator allows you to loop through a state or states until success conditions are met. 
   





