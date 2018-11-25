package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class B2CDcMotor {

    //Declare OpMode Members
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor armDrive;

    //Initialize hardware variables
    leftDrive = HardwareMap.get (DcMotor.class, "left_drive");
    rightDrive = HardwareMap.get (DcMotor.class, "right_drive");
    armDrive = HardwareMap.get (DcMotor.class,"arm_drive");

    //Set motor directions
    leftDrive.setDirection (DcMotor.Direction.FORWARD);
    rightDrive.setDirection(DcMotor.Direction.REVERSE);
    armDrive.setDirection(DcMotor.Direction.REVERSE);

    public Robot()
    {   }
}
