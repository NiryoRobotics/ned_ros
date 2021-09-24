September release - New features batch
===========================================================

Features
-----------------------------------------------------------

Tool commander package
***********************************************************

- TCP service settings
  
  TCP.msg

  SetTCP.srv


Arm commander package
***********************************************************

- New movements available in ArmMoveCommand.msg

    linear pose

    shift linear pose

    trajectory


Python ROS Wrapper package
****************************  

- New movement functions available

    move linear pose

    linear pose

    jog pose shift

    jog joints shift

    shift linear pose

    execute trajectory from pose

- New TCP functions available

    set_tcp

    enable_tcp

    reset_tcp  

- New camera settings functions available

    set_brightness

    set_contrast

    set_saturation

Improvements
---------------------------

- Refactoring Tool Commander and Robot Commander packages.

    Remove Robot Commander package

    Reorder Robot Commander package between Tool Commander and Arm Commander packages. 

- Self collision detection

    Add self-collision detection via MoveIt.

- Collision detection

    Collision detection improvement on each joints.

    Learning mode activation in case of a collision. 