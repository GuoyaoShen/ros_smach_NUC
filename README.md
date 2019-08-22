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

### Trigger List:
    trigger:  no one near, but someone identified far away and facing robot
    action: face person  wave and beckon

    trigger: closest person is walking directly to robot (getting closer with no rotation in base)
    action: facial expression?

    trigger: person is close and facing and person is short (e.g. a child)
    action: bend forward - look down facial expression  ( blink heart in the eyes )
    out trigger: - something so it doesn’t continuously do this. do it only once a minute.

    trigger: closest person turns away
    action: waves good-bye and finds next closest person

    trigger: person is looking squarely at camera and close
    action:  face blinks at person - test initiate face mimic - tilt eye’s


    trigger: if mimic initiated - if person’s face tilts
    action: face (e.g. eye angle) copies tilt of persons head

    trigger: if face mimicking - if arms move out
    action: mimic arm motions.

    trigger: if face mimicking - if person moves closer or farther
    action: waist mimics forward/backward motion.

    trigger: if person quickly moves towards robot and is close
    action: waist motion quickly backwards - and arms thrown forward and out (e.g. surprise)


    trigger: if more than X number of people - and not danced recently
    action: do dance.

    trigger: if no people sensed
    action: sleep mode - lean forward - face off arms - slight forward

    trigger: if sleep mode and person gets near
    action: wakeup action - stand straight - blink eyes several times - go to idle mode

    trigger: default after any action.
    action: idle mode - occasionally blink - turn face to closest person then turn body - waist straight, arms at side.
