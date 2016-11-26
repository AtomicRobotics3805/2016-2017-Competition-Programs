package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.navigationMecanumPID;


@Autonomous(name = "RED autonomous 2")
//@Disabled
public class RED_AutonomousProgramLeaguePlay_V4 extends OpMode {
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

    private DeviceInterfaceModule DIM;

    //1: 1 = moveForward, 2 = moveBackward, 3 = rotateCW, 4 = rotateCCW, 5 = moveLeft, 6 = moveRight, 10 = end navigation, anything else will pause the program until forceNextMovement() is called
    //2: Tp: the speed at which the robot goes when moving in the correct direction.
    //3: Either the wanted angle, or the goal distance in inches.
    private double[][] movementArray = new double[][]{
            //_,______,______}
            {11,    0,     0}, //Shoot
            {1,   0.5,     2}, //Move forwards
            {3,   0.5,    48}, //Turn towards line at 45
            {1,   0.5,    42}, //Move forwards until near line
            {12,    0,     0}, //Approach line
            {3,   0.5,    90}, //Turn towards beacon
            {13,    0,     0}, //Slide to beacon and score
            {10,    0,     0} //Stop all movements
    };

    //Variables for line follower
    private double integral;
    private double setPoint = 0.35;
    private double preError;
    private double Kp = 0.75;
    private double Ki = 0.0003;
    private double Kd = 0;
    private double Tp = 0.1;

    //State Machine variables
    private int mainProgramStep = 0;
    private int scoreBeaconsStep = 0;
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

        leftBeaconServo.setDirection(Servo.Direction.REVERSE);

        leftBeaconServo.setPosition(0);
        rightBeaconServo.setPosition(0);

        particleServo.setPosition(0);
        forkReleaseServo.setPosition(0);

        //IMU navigation
        testNavigator = new navigationMecanumPID(movementArray, this, "AG", leftFront, rightFront, leftBack, rightBack, 1120, 4);
        testNavigator.tuneGains(0.01, 0.00001, 0.02);

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
            case 0: //Shoot
                switch (shootStep) {
                    case 0: //Shoot first particle
                        if (shooterMotor.getCurrentPosition() >= 1680) { //1680 is 1 rev of a NeverRest 60
                            shooterMotor.setPower(0);
                            shootStep++;
                        } else {
                            shooterMotor.setPower(1);
                        }
                        break;
                    case 1: //Load second particle
                        switch (particleReloadStep) {
                            case 0: //Move particle to shooter
                                if (iLoop < 100) { //Loop for timing
                                    //Hold servo out
                                    particleServo.setPosition(0.9);
                                    iLoop++;
                                } else {
                                    iLoop = 0;
                                    particleReloadStep++;
                                }
                                break;
                            case 1: //Reverse shooter to guard ball
                                if (shooterMotor.getCurrentPosition() > 800) { //If the shooter has not yet reached its goal position, or if the timer still has time
                                    shooterMotor.setPower(-0.1);
                                    iLoop++;
                                } else {
                                    //Stop motor for launcher
                                    shooterMotor.setPower(0);
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
                                if (iLoop < 100) { //Loop for timing
                                    shooterMotor.setPower(-0.1);
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
            case 1: //Navigate
                if (testNavigator.navigationType() == 12) {
                    mainProgramStep++;
                } else {
                    telemetry.addData(">>", "Navigating");
                    telemetry.addData("Type", testNavigator.navigationType());
                    testNavigator.loopNavigation();
                }
                break;
            case 2: //Approach line
                if (lineLightSensor.getLightDetected() > setPoint) { //If detected white line
                    testNavigator.forceNextMovement();
                    mainProgramStep++;
                } else {
                    telemetry.addData(">>", "Waiting for line");
                    testNavigator.moveNoStop(0.2, 0); //Move forwards at 20% power
                }
                break;
            case 3: //Navigate
                if (testNavigator.navigationType() == 13) {
                    mainProgramStep++;
                } else {
                    telemetry.addData(">>", "Navigating");
                    telemetry.addData("Type", testNavigator.navigationType());
                    testNavigator.loopNavigation();
                }
                break;
            case 4: //Score the beacons
                telemetry.addData(">>", "Beacons");
                switch (scoreBeaconsStep) {
                    case 0: //Slide against line
                        double lightDetected = lineLightSensor.getLightDetected();
                        if (lightDetected > setPoint) { //If detected white line
                            leftFront.setPower(0);
                            leftBack.setPower(0);
                            rightFront.setPower(0);
                            rightBack.setPower(0);
                            scoreBeaconsStep++;
                        } else {
                            telemetry.addData(">>>", "Waiting for line");
                            testNavigator.moveNoStop(0, 0.5); //Move left at 50% power
                        }
                        break;
                    case 1: //Follow line
                        double distanceIR = 27.86 * Math.pow(sharpIR.getVoltage(), -1.15);
                        if (distanceIR <= 15) {
                            testNavigator.moveNoStop(0.2, 0);

                            if (iLoop < 250) { //Loop to allow the servos enough time, as well as detect the color
                                //Hold the servo out dependant on the color
                                scoreBeacon();
                                iLoop++;
                            } else {
                                leftBeaconServo.setPosition(0);
                                rightBeaconServo.setPosition(0);
                                scoreBeaconsStep++;
                            }
                        } else {
                            telemetry.addData(">>>", "Following line");
                            telemetry.addData("Distance from wall", "%1$s", distanceIR);
                            lineFollow();
                        }
                        break;
                    case 2:
                        testNavigator.forceNextMovement();
                        mainProgramStep++;
                        break;
                }
                break;
            case 5: //Navigate
                telemetry.addData(">>", "Navigating");
                telemetry.addData("Type", testNavigator.navigationType());
                testNavigator.loopNavigation();
                break;
        }
    }

    private short scoreBeacon() {
        telemetry.addData("Color", "RGB: %1$s, %2$s, %3$s", beaconColorSensor.red(), beaconColorSensor.green(), beaconColorSensor.blue());

        if (beaconColorSensor.red() > beaconColorSensor.blue() && beaconColorSensor.red() > beaconColorSensor.green()) {
            DIM.setLED(1, true);           //Red ON
            DIM.setLED(0, false);          //Blue OFF
            leftBeaconServo.setPosition(0);
            rightBeaconServo.setPosition(1);
            telemetry.addData("Beacon", "RED");
            return 1;
        } else if (beaconColorSensor.blue() > beaconColorSensor.red() && beaconColorSensor.blue() > beaconColorSensor.green()) {
            DIM.setLED(1, false);          //Red OFF
            DIM.setLED(0, true);           //Blue ON
            leftBeaconServo.setPosition(1);
            rightBeaconServo.setPosition(0);
            telemetry.addData("Beacon", "BLUE");
            return 2;
        } else {
            DIM.setLED(1, false);           //Red OFF
            DIM.setLED(0, false);           //Blue OFF
            leftBeaconServo.setPosition(0);
            rightBeaconServo.setPosition(0);
            telemetry.addData("Beacon", "Not Detected");
            return 0;
        }
    }

    private void lineFollow() {
        double error = lineLightSensor.getLightDetected() - setPoint;
        integral += error;
        double derivative = error - preError;
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        double leftOut = Range.clip(Tp - output, -0.5, 0.5);
        double rightOut = Range.clip(Tp + output, -0.5, 0.5);

        telemetry.addData("Motor output", "%1$s, %2$s", leftOut, rightOut);
        leftFront.setPower(leftOut);
        leftBack.setPower(leftOut);
        rightFront.setPower(rightOut);
        rightBack.setPower(rightOut);
        preError = error;
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
