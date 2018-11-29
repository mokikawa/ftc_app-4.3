package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class B2CDcMotor {

    //Declare OpMode Members
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor armDrive = null;
    Telemetry telemetry;
    HardwareMap hwMap;

    public B2CDcMotor (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init (HardwareMap ahMap) {
        hwMap = ahMap;
        //Initialize hardware variables
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive =hwMap.get(DcMotor.class, "right_drive");
        armDrive = hwMap.get(DcMotor.class, "arm_drive");

        //Set motor directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setDirection(DcMotor.Direction.REVERSE);
        
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //Set motor power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armDrive.setPower(0);
    }

	//Methods
    private void stopRobot() {
	    leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	    rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	    armDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	    
	    leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	    rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	    armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	    
	    leftDrive.setPower(0);
	    rightDrive.setPower(0);
	    armDrive.setPower(0);
    }
 
}
