package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.navigationMecanumPID;


@Autonomous(name = "BLUE & RED shoot")
//@Disabled
public class ShootOnlyAutonomous extends OpMode {
    private navigationMecanumPID testNavigator;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    private DcMotor shooterMotor;

    private LightSensor lineLightSensor;
    private ColorSensor beaconColorSensor;
    private AnalogInput sharpIR;

    private Servo rightBeaconServo;
    private Servo leftBeaconServo;
    private Servo particleServo;
    private Servo forkReleaseServo;
    private Servo collectionReleaseServo;

    private DeviceInterfaceModule DIM;

    //1: 1 = moveForward, 2 = moveBackward, 3 = rotateCW, 4 = rotateCCW, 5 = moveLeft, 6 = moveRight, 10 = end navigation, anything else will pause the program until forceNextMovement() is called
    //2: Tp: the speed at which the robot goes when moving in the correct direction.
    //3: Either the wanted angle, or the goal distance in inches.
    private double[][] movementArray = new double[][]{
           //_,______,______}
            {11,    0,     0}, //Guard ball
            {1,   0.5,    24}, //Move forwards
            {12,    0,     0}, //Shoot
            {1,   0.5,    48}, //Move forwards and disrupt ball
            {10,    0,     0} //Stop all movements
    };

    //State Machine variables
    private int mainProgramStep = 0;
    private int shootStep = 0;

    private int iLoop = 0;
    private int particleReloadStep;

    @Override
    public void init() {
        //Motors
        rightFront = hardwareMap.dcMotor.get("RF");
        rightBack = hardwareMap.dcMotor.get("RB");
        leftFront = hardwareMap.dcMotor.get("LF");
        leftBack = hardwareMap.dcMotor.get("LB");

        shooterMotor = hardwareMap.dcMotor.get("SM");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        //Sensors
        lineLightSensor = hardwareMap.lightSensor.get("LS");
        beaconColorSensor = hardwareMap.colorSensor.get("CS");
        sharpIR = hardwareMap.analogInput.get("SIR");

        beaconColorSensor.enableLed(false);
        lineLightSensor.enableLed(true);

        //Servos
        rightBeaconServo = hardwareMap.servo.get("RBS");
        leftBeaconServo = hardwareMap.servo.get("LBS");

        particleServo = hardwareMap.servo.get("PS");
        forkReleaseServo = hardwareMap.servo.get("FRS");
        collectionReleaseServo = hardwareMap.servo.get("CRS");

        leftBeaconServo.setDirection(Servo.Direction.REVERSE);

        leftBeaconServo.setPosition(0);
        rightBeaconServo.setPosition(0);

        particleServo.setPosition(0);
        forkReleaseServo.setPosition(0);
        collectionReleaseServo.setPosition(0);

        //IMU navigation
        testNavigator = new navigationMecanumPID(movementArray, this, "AG", leftFront, rightFront, leftBack, rightBack, 1120, 4);
        testNavigator.tuneGains(0.035, 0.00001, 0.04); //0.01, 0.00001, 0.02

        //Device Interface Module
        DIM = hardwareMap.deviceInterfaceModule.get("Device Interface Module");

        telemetry.addData(">", "Press start when ready...");

        //Set the motor modes
        //Reset
        resetDriveEncoders();
        resetEncoder(shooterMotor);
        //Set main mode
        runWithoutDriveEncoders();
        runWithoutEncoder(shooterMotor);
    }

    @Override
    public void start() {
        testNavigator.initialize();
    }

    @Override
    public void loop() {
        switch (mainProgramStep) {
            case 0: //Hold ball before navigating
                switch (particleReloadStep) {
                    case 0: //Reverse shooter to guard ball
                        if (shooterMotor.getCurrentPosition() < 850) { //If the shooter has not yet reached its goal position
                            shooterMotor.setPower(-0.1);
                        } else {
                            //Stop motor for launcher
                            shooterMotor.setPower(0);
                            particleReloadStep++;
                        }
                        break;
                    case 1: //Squeeze ball
                        if (iLoop < 150) { //Loop for timing
                            shooterMotor.setPower(-0.05);
                            iLoop++;
                        } else {
                            iLoop = 0;
                            particleReloadStep++;
                        }
                        break;
                    case 2:
                        collectionReleaseServo.setPosition(0.5);
                        particleReloadStep = 0;
                        testNavigator.forceNextMovement();
                        mainProgramStep++;
                        break;
                }
                break;
            case 1: //Navigate
                if (testNavigator.navigationType() == 12) {
                    mainProgramStep++;
                } else {
                    telemetry.addData(">>", "Navigating");
                    telemetry.addData("Type", testNavigator.navigationType());
                    testNavigator.loopNavigation();
                }
                break;
            case 2: //Shoot
                switch (shootStep) {
                    case 0: //Shoot first particle
                        if (shooterMotor.getCurrentPosition() >= 1680) { //1680 is 1 rev of a NeverRest 60 relative to start position.
                            shooterMotor.setPower(0);
                            shootStep++;
                        } else {
                            shooterMotor.setPower(1);
                        }
                        break;
                    case 1: //Load second particle
                        switch (particleReloadStep) {
                            case 0: //Reverse shooter to guard ball
                                if (shooterMotor.getCurrentPosition() > 850) { //If the shooter has not yet reached its goal position
                                    shooterMotor.setPower(-0.1);
                                    iLoop++;
                                } else {
                                    //Stop motor for launcher
                                    shooterMotor.setPower(0);
                                    iLoop = 0;
                                    particleReloadStep++;
                                }
                                break;
                            case 1: //Move particle to shooter
                                if (iLoop < 200) { //Loop for timing
                                    //Hold servo out
                                    particleServo.setPosition(1);
                                    iLoop++;
                                } else {
                                    iLoop = 0;
                                    particleReloadStep++;
                                }
                                break;
                            case 2: //Move arm back to home position
                                if (iLoop < 100) { //Loop for timing
                                    //Return servo to home
                                    particleServo.setPosition(0);
                                    iLoop++;
                                } else {
                                    iLoop = 0;
                                    particleReloadStep++;
                                }
                                break;
                            case 3: //Squeeze ball
                                if (iLoop < 150) { //Loop for timing
                                    shooterMotor.setPower(-0.05);
                                    iLoop++;
                                } else {
                                    iLoop = 0;
                                    particleReloadStep++;
                                }
                                break;
                            case 4:
                                shootStep++;
                                break;
                        }
                        break;
                    case 2: //Shoot second particle
                        if (shooterMotor.getCurrentPosition() >= 3360) { //3360 is 2 revs of a NeverRest 60
                            shooterMotor.setPower(0);
                            shootStep++;
                        } else {
                            shooterMotor.setPower(1);
                        }
                        break;
                    case 3: //Move on to navigation
                        testNavigator.forceNextMovement();
                        mainProgramStep++;
                        break;
                }
                break;
            case 3: //Navigate
                telemetry.addData(">>", "Navigating");
                telemetry.addData("Type", testNavigator.navigationType());
                testNavigator.loopNavigation();
                break;
        }
    }


    private void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void runWithoutEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void runToPosition(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runUsingEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetDriveEncoders() {
        resetEncoder(leftFront);
        resetEncoder(rightFront);
        resetEncoder(leftBack);
        resetEncoder(rightBack);
    }

    private void runWithoutDriveEncoders() {
        runWithoutEncoder(leftFront);
        runWithoutEncoder(rightFront);
        runWithoutEncoder(leftBack);
        runWithoutEncoder(rightBack);
    }
}
