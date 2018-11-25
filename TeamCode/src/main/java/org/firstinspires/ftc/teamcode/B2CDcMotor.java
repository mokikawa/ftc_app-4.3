package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class B2CDcMotor {

    //Declare OpMode Members
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor armDrive = null;

    //Initialize hardware variables
    leftDrive = HardwareMap.get (DcMotor.class, "left_drive");
    rightDrive = HardwareMap.get (DcMotor.class, "right_drive");
    armDrive = HardwareMap.get (DcMotor.class,"arm_drive");

    public B2CDcMotor (Telemetry telemetry) {
        this.Telemetry = telemetry;
    }

    //Set motor directions
    leftDrive.setDirection (DcMotor.Direction.FORWARD);
    rightDrive.setDirection(DcMotor.Direction.REVERSE);
    armDrive.setDirection(DcMotor.Direction.REVERSE);

    public Robot()
    {   }
}
