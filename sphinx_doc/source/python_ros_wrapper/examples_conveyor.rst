Examples: Conveyor Belt
========================

This document shows how to use Ned's Conveyor Belt.

| If you want see more about Ned's Conveyor Belt functions, you can look at :ref:`API - Conveyor<source/python_ros_wrapper/ros_wrapper_doc:Conveyor Belt>`.

.. note::
    Imports & initialisation are not mentioned, but you should not forget it!

Simple Conveyor Belt control
-------------------------------
This short example shows how to connect a Conveyor Belt, activate the connection and
launch its motor: ::

    niryo_robot = NiryoRosWrapper()

    # Activating connexion with conveyor and storing ID
    conveyor_id = niryo_robot.set_conveyor()

    # Running conveyor at 50% of its maximum speed, in Forward direction
    niryo_robot.control_conveyor(conveyor_id, True, 100, ConveyorDirection.FORWARD)

    # Stopping robot motor
    niryo_robot.control_conveyor(conveyor_id, True, 0, ConveyorDirection.FORWARD)

    # Deactivating connexion with conveyor
    niryo_robot.unset_conveyor(conveyor_id)


Advanced Conveyor Belt control
-------------------------------
This example shows how to do a certain amount of pick & place by using
the Conveyor Belt with the infrared sensor: ::


    def run_conveyor(robot, conveyor):
        robot.control_conveyor(conveyor, bool_control_on=True,
                               speed=50, direction=ConveyorDirection.FORWARD)

    # -- Setting variables
    sensor_pin_id = PinID.GPIO_1A

    catch_nb = 5

    # The pick pose
    pick_pose = [0.25, 0., 0.15, 0., 1.57, 0.0]
    # The Place pose
    place_pose = [0.0, -0.25, 0.1, 0., 1.57, -1.57]

    # -- MAIN PROGRAM

    niryo_robot = NiryoRosWrapper()

    # Activating connexion with conveyor
    conveyor_id = niryo_robot.set_conveyor()

    for i in range(catch_nb):
        run_conveyor(niryo_robot, conveyor_id)
        while niryo_robot.digital_read(sensor_pin_id) == PinState.LOW:
            niryo_robot.wait(0.1)

        # Stopping robot motor
        niryo_robot.control_conveyor(conveyor_id, True, 0, ConveyorDirection.FORWARD)
        # Making a pick & place
        niryo_robot.pick_and_place(pick_pose, place_pose)

    # Deactivating connexion with conveyor
    niryo_robot.unset_conveyor(conveyor_id)

