# ros_smach_NUC
* A [smach](http://wiki.ros.org/smach) architecture of ros on NUC.
* Communication between nodes is based on publish-subscribe.

### TO DO:
* test passing data between node and node of a state machine (DONE)
* test looping in one state in a state machine (DONE)
* test concurrence container for condition judgement (DONE)
* decide all msg types in NUC

### Notes:
* Concurrence container do "execute()" function for every inner state first every loop. So DO NOT init Subscriber in Concurrence Container and pass data to states because this cannot get right msg. Just init subscriber in "__init__()" function of every state.
* DO NOT define callback function for subscriber outside of state which has subscriber in it, won't pass msg data correctly, because smach isolate data access strictly.

### Architecture of Smach
![smach architecture](https://github.com/GuoyaoShen/ros_smach_NUC/blob/master/smach_picture/smach_v2.2.jpg "smach architecture")
