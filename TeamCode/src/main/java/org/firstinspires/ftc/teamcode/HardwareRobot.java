package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class HardwareRobot {
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor lift = null;
    public int liftSpeed = 1;

    public DcMotor firstArm = null;
    public DcMotor middleArm = null;

    public Servo leftServo = null;
    public Servo rightServo = null;
    public Servo liftServo = null;
    double servoSpeed = .0005;
    public double servoHomePosition = 0.5;

    public Servo markerServo = null;

    int direction;

    double distance;

    public boolean extendedMarker = false;

    public boolean isTopElbow = false;
    public boolean isTopArm = false;

    public DistanceSensor sensorRange;
    //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
    //public  Servo servo = null;
    public boolean ishome = true;
    BNO055IMU imu;
    Orientation angles;
    HardwareMap hwMap = null;
    Telemetry telemetry;
    RobotConstants constants = new RobotConstants();
    public boolean isTop = false;

    public GoldAlignDetector detector;
    int centerValue = 290;



    public HardwareRobot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // assign devices to config
        leftFront = hwMap.get(DcMotor.class, "motor_1");
        leftBack = hwMap.get(DcMotor.class, "motor_2");
        rightBack = hwMap.get(DcMotor.class, "motor_3");
        rightFront = hwMap.get(DcMotor.class, "motor_4");

        firstArm = hwMap.get(DcMotor.class, "firstArm");
        middleArm = hwMap.get(DcMotor.class, "middleArm");

        lift = hwMap.get(DcMotor.class, "lift");

        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");

        leftServo = hwMap.get(Servo.class, "leftServo");
        rightServo = hwMap.get(Servo.class, "rightServo");
        liftServo = hwMap.get(Servo.class, "liftServo");

        markerServo = hwMap.get(Servo.class, "markerServo");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotor.Direction.REVERSE); //changed direction
        /* servo = hwMap.get(Servo.class, "servo_1");
        //set motor direction*/
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        firstArm.setDirection(DcMotorSimple.Direction.REVERSE);
        middleArm.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        firstArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set motor power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        firstArm.setPower(0);
        middleArm.setPower(0);

        lift.setPower(0);

        //set servo to initial position
        /*servo.setPosition(servoHomePosition);*/
        liftServo.setPosition(.3);
        leftServo.setPosition(constants.getLeftServoClose());
        rightServo.setPosition(constants.getRightServoClose());

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        detector = new GoldAlignDetector();
        detector.init(hwMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 150; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

    }

    //Methods

    private void stopRobot() {
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

    }

    public void move(double distance, double speed) {
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //find how many encoder counts the motor is at, then add the distance to it
        newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_40());
        newLeftBackTarget = leftBack.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_40());
        newRightFrontTarget = rightFront.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_40());
        newRightBackTarget = rightBack.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_40());
        //set the target encoder count to the motors
        leftFront.setTargetPosition(newLeftFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        rightBack.setTargetPosition(newRightBackTarget);
        //set mode to run to position
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set speed
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed));
        rightBack.setPower(Math.abs(speed));
        //While loop is necessary!
        while (leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy() || leftFront.isBusy())
        {
            telemetry.addData("Left Front:", leftFront.getCurrentPosition());
            telemetry.addData("Left Back:", leftBack.getCurrentPosition());
            telemetry.addData("Right Front:", rightFront.getCurrentPosition());
            telemetry.addData("Right Back:", rightBack.getCurrentPosition());

            telemetry.update();

        }

        stopRobot();
    }

// TURN DOES NOT LIKE NEGATIVE,COUNTER CLOCKWISE IS POS

    public void turn(double angle, double speed) {
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double initalAngle = angles.firstAngle;
        double motorPower;
        double minMotorPower = 0.2;
        double powerScaleFactor;
        double targetAngle;
        double currentAngle;
        double deltaAngle;
        double robotAngle = angles.firstAngle;
        double previousAngle = angles.firstAngle;

        targetAngle = initalAngle + angle;

        while (Math.abs(targetAngle - robotAngle)> .25)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = angles.firstAngle;

            //update speed dynamically to slow when approaching the target
            powerScaleFactor = Math.sqrt(Math.abs(targetAngle-robotAngle)/angle);
            motorPower = powerScaleFactor*speed;
            if (motorPower < minMotorPower)
            {
                motorPower = minMotorPower;
            }

            //determine which direction the robot should turn



            if ((targetAngle - robotAngle) > 0) {
                leftBack.setPower(-motorPower);
                leftFront.setPower(-motorPower);
                rightBack.setPower(motorPower);
                rightFront.setPower(motorPower);
            } else {
                leftBack.setPower(motorPower);
                rightBack.setPower(-motorPower);
                leftFront.setPower(motorPower);
                rightFront.setPower(-motorPower);
            }


            //define how the angle is changing and deal with the stupid 180 -> -180 thing
            deltaAngle = currentAngle - previousAngle;
            if (deltaAngle > 180)
            {
                deltaAngle -= 360;
            }
            else if(deltaAngle < -180)
            {
                deltaAngle += 360;
            }

            robotAngle += deltaAngle;
            previousAngle = currentAngle;

            telemetry.addData("robotangle", robotAngle);
            telemetry.addData("deltaAngle", deltaAngle);
            telemetry.addData("currentAngle", currentAngle);

            telemetry.update();

        }
        stopRobot();
    }

    public double getDistance(){
        int count = 10;

        for (int i = 0; i < count ; i++) {
            distance =+ sensorRange.getDistance(DistanceUnit.INCH);
        }
        distance = distance / count;

        return distance;
    }

    public void strafe(double distance, double speed){
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(distance * constants.getStrafe());
        newLeftBackTarget = leftBack.getCurrentPosition() - (int)(distance * constants.getStrafe());
        newRightFrontTarget = rightFront.getCurrentPosition() + (int)(distance * constants.getStrafe());
        newRightBackTarget = rightBack.getCurrentPosition() - (int)(distance * constants.getStrafe());

        leftFront.setTargetPosition(newLeftFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed)); //-
        rightBack.setPower(Math.abs(speed)); //-

        while (leftBack.isBusy() || rightFront.isBusy() || rightBack.isBusy() || leftFront.isBusy())
        {
            telemetry.addData("Left Front:", leftFront.getCurrentPosition());
            telemetry.addData("Left Back:", leftBack.getCurrentPosition());
            telemetry.addData("Right Front:", rightFront.getCurrentPosition());
            telemetry.addData("Right Back:", rightBack.getCurrentPosition());

            telemetry.update();
        }
        stopRobot();
    }

    public void lift(double speed){
        int newLiftTarget;

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (isTop) {
            newLiftTarget = 0;
            isTop = false;


        } else {
            newLiftTarget = (int) constants.getLift();
            isTop = true;
        }

        lift.setTargetPosition(newLiftTarget);

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setPower(speed);

        while (lift.isBusy()){
            if (isTop) {
                telemetry.addData("Going Up", 1);
            }else{
                telemetry.addData("Going Down", 1);
            }
            telemetry.addData("current:",lift.getCurrentPosition());
            telemetry.addData("target:", newLiftTarget);
            telemetry.update();
        }
        lift.setPower(0);

    }
    public void liftUp(double speed){
        int newLiftTarget;

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        newLiftTarget = lift.getCurrentPosition() + liftSpeed;

        lift.setTargetPosition(newLiftTarget);

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setPower(speed);
    }
    public void liftDown(double speed){
        int newLiftTarget;

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        newLiftTarget = lift.getCurrentPosition() - liftSpeed;

        lift.setTargetPosition(newLiftTarget);

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setPower(speed);
    }

    public void firstArm(double speed){
        int newFirstArmTarget;

        firstArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        firstArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        firstArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (isTopArm) {
            newFirstArmTarget = firstArm.getCurrentPosition() - (int) constants.getArm();
            isTopArm = false;
            speed = -speed;

        } else {
            newFirstArmTarget = firstArm.getCurrentPosition() + (int) constants.getArm();
            isTopArm = true;
        }

        firstArm.setTargetPosition(newFirstArmTarget);

        firstArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        firstArm.setPower(speed);

        while (firstArm.isBusy()){
        }
        firstArm.setPower(0);
    }
    public void middleArm(double speed){
        int newFirstArmTarget;

        middleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        middleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        middleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (isTopElbow) {
            newFirstArmTarget = middleArm.getCurrentPosition() - (int) constants.getElbow();
            isTopElbow = false;
            speed = -speed;

        } else {
            newFirstArmTarget = middleArm.getCurrentPosition() + (int) constants.getElbow();
            isTopElbow = true;
        }

        middleArm.setTargetPosition(newFirstArmTarget);

        middleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        middleArm.setPower(speed);

        while (middleArm.isBusy()){
        }
        middleArm.setPower(0);
    }


    public void alignRobot(double speed){
        while (!detector.getAligned()){
            if (detector.getXPosition() > centerValue){
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(speed);
            }
            if (detector.getXPosition() < centerValue){
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(speed);
            }
        }
        if (detector.getAligned()){
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
    }

    public void gyroMove(double distance, double speed) {
        double deltaSpeed = 0.05;// hardcoded
        double deltaA = 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;

        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //find how many encoder counts the motor is at, then add the distance to it
        newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_40());
        newLeftBackTarget = leftBack.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_40());
        newRightFrontTarget = rightFront.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_40());
        newRightBackTarget = rightBack.getCurrentPosition() + (int)(distance * constants.getTICKS_PER_INCH_40());
        //set the target encoder count to the motors
        leftFront.setTargetPosition(newLeftFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        rightBack.setTargetPosition(newRightBackTarget);
        //set mode to run to position
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set speed
        leftFront.setPower(Math.abs(speed));
        rightFront.setPower(Math.abs(speed));
        leftBack.setPower(Math.abs(speed));
        rightBack.setPower(Math.abs(speed));
        //While loop is necessary!
        while (leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy() && leftFront.isBusy())
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            if (currentAngle - angles.firstAngle > deltaA){
                telemetry.addData("left", 0);
                leftFront.setPower(speed - deltaSpeed);
                rightFront.setPower(speed + deltaSpeed);
                leftBack.setPower(speed - deltaSpeed);
                rightBack.setPower(speed + deltaSpeed);
            }else if (currentAngle - angles.firstAngle < deltaA){
                telemetry.addData("right", 0);
                leftFront.setPower(speed + deltaSpeed);
                rightFront.setPower(speed - deltaSpeed);
                leftBack.setPower(speed + deltaSpeed);
                rightBack.setPower(speed - deltaSpeed);
            }else {
                telemetry.addData("straight", 0);
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftBack.setPower(speed);
                rightBack.setPower(speed);
            }
            telemetry.update();
        }
        stopRobot();
    }
//    public void leftServoOpen(){
//        leftServo.setPosition(leftServo.getPosition() + .0003);
//    }
//    public void leftServoClose(){
//        leftServo.setPosition(leftServo.getPosition() - .0003);
//    }
//    public void rightServoOpen(){
//        rightServo.setPosition(rightServo.getPosition() + .0003);
//    }
//    public void rightServoClose(){
//        rightServo.setPosition(rightServo.getPosition() - .0003);
//    }
//
//    public void liftServoOpen(){
//        liftServo.setPosition(liftServo.getPosition() + .0003);
//    }
//    public void liftServoClose(){
//        liftServo.setPosition(liftServo.getPosition() - .0003);
//    }


    public void leftServoOpen(){
        leftServo.setPosition(constants.getLeftServoOpen());
    }
    public void leftServoClose(){
        leftServo.setPosition(constants.getLeftServoClose());
    }
    public void rightServoOpen(){
        rightServo.setPosition(constants.getRightServoOpen());
    }
    public void rightServoClose(){
        rightServo.setPosition(constants.getRightServoClose());
    }
    public void liftServoOpen(){
        liftServo.setPosition(constants.getLiftServoOpen());
    }
    public void liftServoClose(){
        liftServo.setPosition(constants.getLiftServoClose());
    }

    public void sampleMove2(){
        gyroMove(-16, 1);
        strafe(14.5,.5);



    }

    public void sampleMove(){
        // direction: -1 is left / 0 is center / 1 is right
        double goldPos = 0;
        int scanAmount = 10;
        double forwardMovement = 10;
//        gyroMove(-18,1);

        int i = 0;
        while (i < scanAmount){
            if (detector.isFound())
            {
                goldPos += detector.getXPosition();
                i++;
            }
        }

        /*
        for (int i = 0; i <scanAmount ; i++) {
            goldPos += detector.getXPosition();

        }
        */
        goldPos = goldPos / scanAmount;

        if (goldPos > centerValue && !detector.getAligned()){
            direction = 1;
            gyroMove(-16,1);
            strafe(+14.5, .5);
            gyroMove(-forwardMovement, .5);
            gyroMove(forwardMovement, .5);
        }
        else if (goldPos < centerValue && !detector.getAligned()){
            direction = -1;
            gyroMove(-16,1);
            strafe(-14.5, .5);
            gyroMove(-forwardMovement, .5);
            gyroMove(forwardMovement, .5);
        }
        else if (detector.getAligned()){
            direction = 0;
            gyroMove(-16,1);
            gyroMove(-forwardMovement, .5);
            gyroMove(forwardMovement,.5);
        }


        direction = (int)goldPos;
    }

    public void markerServo(){
        if (extendedMarker = false){
            markerServo.setPosition(1);
            extendedMarker = true;
        }
        if (extendedMarker){
            markerServo.setPosition(0);
            extendedMarker = false;
        }
    }

    public void armDown(double speed){
        int newMiddleArmTarget = 0;

        middleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        middleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (middleArm.getCurrentPosition() > constants.getElbowMax()){
            newMiddleArmTarget += (int) constants.getElbow() / 2;
        }

        middleArm.setTargetPosition(newMiddleArmTarget);

        middleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        middleArm.setPower(speed);

        while (middleArm.isBusy()){
        }
        middleArm.setPower(0);

    }
    public void armUp(double speed){
        int newMiddleArmTarget = 0;

        middleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        middleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (middleArm.getCurrentPosition() < constants.getElbowMin()){
            newMiddleArmTarget -= (int) constants.getElbow() / 2;
        }

        middleArm.setTargetPosition(newMiddleArmTarget);

        middleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        middleArm.setPower(speed);

        while (middleArm.isBusy()){
        }
        middleArm.setPower(0);

    }

}