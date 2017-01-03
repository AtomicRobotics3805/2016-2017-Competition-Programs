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


@Autonomous(name = "RED")
//@Disabled
public class RED_AutonomousProgram_V6 extends OpMode {
    private navigationMecanumPID testNavigator;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    private DcMotor shooterMotor;

    private LightSensor lineLightSensor;
    private ColorSensor beaconColorSensor;
    private AnalogInput sharpIR;
    private AnalogInput ultrasonicSensor;

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
            {11,    0,     0}, //Shoot
            {1,   0.75,     6}, //Move forwards 6 inches
            {3,   0.75,    25}, //Turn towards beacons at 25
            {1,   0.75,    48}, //Move forwards 4 feet 6 inches (48)
            {3,   0.75,    90}, //Align parallel with lines (90)
            {1,  0.25,     4}, //Move forwards 4 inches to straighten out
            {12,    0,     0}, //Score first beacon
            {2,  -0.75,    -6}, //Back up 6 inches
            {5,     1,     6}, //Slide over to move the light sensor away form the line
            {13,    0,     0}, //Slide and score second beacon
            {10,    0,     0}  //Stop all movements
    };

    //Variables for line follower
    private double integral;
    private double setPoint = 0.35;
    private double preError;
    private double Kp = 1.274;
    private double Ki = 0.0019;
    private double Kd = 1.0;
    private double Tp = 0.125;

    //Variables for wall follow
    private double distSetPoint = 38.1;
    private double distIntegral;
    private double distPreError;
    private double distKp = 0.05;
    private double distKi = 0;
    private double distKd = 0;

    //State Machine variables
    private int mainProgramStep = 0;
    private int scoreBeaconsStep = 0;
    private int shootStep = 0;

    private int iLoop = 0;
    private int particleReloadStep;

    private short colorVal;

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
        ultrasonicSensor = hardwareMap.analogInput.get("US");

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
            case 0: //Shoot
                telemetry.addData(">", "Shooting");
                switch (shootStep) {
                    case 0: //Shoot first particle
                        telemetry.addData(">>", "Shooting 1");
                        if (shooterMotor.getCurrentPosition() >= 1680) { //1680 is 1 rev of a NeverRest 60
                            shooterMotor.setPower(0);
                            shootStep++;
                        } else {
                            shooterMotor.setPower(1);
                        }
                        break;
                    case 1: //Load second particle
                        telemetry.addData(">>", "Loading");
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
                        telemetry.addData(">>", "Shooting 2");
                        if (shooterMotor.getCurrentPosition() >= 3360) { //3360 is 2 revs of a NeverRest 60
                            shooterMotor.setPower(0);
                            shootStep++;
                        } else {
                            shooterMotor.setPower(1);
                        }
                        break;
                    case 3: //Release collection and move on to navigation
                        telemetry.addData(">>", "Release collection");
                        collectionReleaseServo.setPosition(0.5);
                        testNavigator.forceNextMovement();
                        mainProgramStep++;
                        break;
                }
                break;
            case 1: //Navigate
                if (testNavigator.navigationType() == 12) {
                    mainProgramStep++;
                } else {
                    telemetry.addData(">", "Navigating");
                    telemetry.addData("Type", testNavigator.navigationType());
                    testNavigator.loopNavigation();
                }
                break;
            case 2: //Score the beacon
                telemetry.addData(">", "Beacon 1");
                switch (scoreBeaconsStep) {
                    case 0:
                        telemetry.addData(">>", "Aligning distance with wall");
                        if (getDistance() > distSetPoint) {
                            testNavigator.moveNoStop(0.3, 0);
                        } else {
                            fulLStop();
                            scoreBeaconsStep++;
                        }
                        break;
                    case 1: //Slide against line
                        telemetry.addData(">>", "Sliding against line");
                        double lightDetected = lineLightSensor.getLightDetected();
                        if (lightDetected > setPoint) { //If detected white line
                            testNavigator.tuneGains(0.035, 0.00001, 0.04);
                            scoreBeaconsStep++;
                        } else {
                            telemetry.addData(">>>", "Waiting for line");
                            testNavigator.tuneGains(0.02, 0.00002, 0.008);
                            testNavigator.moveNoStop(wallFollow(distSetPoint), -0.5); //Move left at 50% power while maintaining a 25 cm distance from the wall
                        }
                        break;
                    case 2:
                        telemetry.addData(">>", "Following line");
                        if (getDistance() < 15) {
                            fulLStop();
                            colorVal = getBeaconColor();
                            scoreBeaconsStep++;
                        } else {
                            lineFollow(true);
                        }
                        break;
                    case 3:
                        telemetry.addData(">>", "Extend button servos");
                        if (iLoop < 100) { //Loop to allow the servos enough time
                            //Hold the servo out dependant on the color
                            switch (colorVal) {
                                case 0:
                                    telemetry.addData(">>>", "Unknown color");
                                    leftBeaconServo.setPosition(1);
                                    rightBeaconServo.setPosition(1);
                                    break;
                                case 1:
                                    telemetry.addData(">>>", "RED");
                                    leftBeaconServo.setPosition(0);
                                    rightBeaconServo.setPosition(1);
                                    break;
                                case 2:
                                    telemetry.addData(">>>", "BLUE");
                                    leftBeaconServo.setPosition(1);
                                    rightBeaconServo.setPosition(0);
                                    break;
                            }
                            iLoop++;
                        } else {
                            iLoop = 0;
                            scoreBeaconsStep++;
                        }
                        break;
                    case 4:
                        if (iLoop < 150) {
                            telemetry.addData(">>", "Following line");
                            lineFollow(true);
                            iLoop++;
                        } else {
                            leftBeaconServo.setPosition(0);
                            rightBeaconServo.setPosition(0);
                            iLoop = 0;
                            fulLStop();
                            scoreBeaconsStep++;
                        }
                        break;
                    case 5:
                        scoreBeaconsStep = 0;
                        testNavigator.forceNextMovement();
                        mainProgramStep++;
                        break;
                }
                break;
            case 3: //Navigate
                if (testNavigator.navigationType() == 13) {
                    mainProgramStep++;
                } else {
                    telemetry.addData(">", "Navigating");
                    telemetry.addData("Type", testNavigator.navigationType());
                    testNavigator.loopNavigation();
                }
                break;
            case 4: //Score second beacon
                telemetry.addData(">", "Beacon 2");
                switch (scoreBeaconsStep) {
                    case 0:
                        telemetry.addData(">>", "Aligning distance with wall");
                        if (getDistance() < distSetPoint) {
                            testNavigator.moveNoStop(-0.3, 0);
                        } else {
                            fulLStop();
                            scoreBeaconsStep++;
                        }
                        break;
                    case 1: //Slide against line
                        telemetry.addData(">>", "Sliding against line");
                        double lightDetected = lineLightSensor.getLightDetected();
                        if (lightDetected > setPoint) { //If detected white line
                            testNavigator.tuneGains(0.035, 0.00001, 0.04);
                            scoreBeaconsStep++;
                        } else {
                            telemetry.addData(">>>", "Waiting for line");
                            testNavigator.tuneGains(0.02, 0.00002, 0.008);
                            testNavigator.moveNoStop(wallFollow(distSetPoint), 1); //Move right at 100% power while maintaining a 25 cm distance from the wall
                        }
                        break;
                    case 2:
                        telemetry.addData(">>", "Following line");
                        if (getDistance() < 15) {
                            fulLStop();
                            colorVal = getBeaconColor();
                            scoreBeaconsStep++;
                        } else {
                            lineFollow(false);
                        }
                        break;
                    case 3:
                        telemetry.addData(">>", "Extend button servos");
                        if (iLoop < 100) { //Loop to allow the servos enough time
                            //Hold the servo out dependant on the color
                            switch (colorVal) {
                                case 0:
                                    telemetry.addData(">>>", "Unknown color");
                                    leftBeaconServo.setPosition(1);
                                    rightBeaconServo.setPosition(1);
                                    break;
                                case 1:
                                    telemetry.addData(">>>", "RED");
                                    leftBeaconServo.setPosition(0);
                                    rightBeaconServo.setPosition(1);
                                    break;
                                case 2:
                                    telemetry.addData(">>>", "BLUE");
                                    leftBeaconServo.setPosition(1);
                                    rightBeaconServo.setPosition(0);
                                    break;
                            }
                            iLoop++;
                        } else {
                            iLoop = 0;
                            scoreBeaconsStep++;
                        }
                        break;
                    case 4:
                        if (iLoop < 150) {
                            telemetry.addData(">>", "Following line");
                            lineFollow(false);
                            iLoop++;
                        } else {
                            leftBeaconServo.setPosition(0);
                            rightBeaconServo.setPosition(0);
                            iLoop = 0;
                            fulLStop();
                            scoreBeaconsStep++;
                        }
                        break;
                    case 5:
                        scoreBeaconsStep = 0;
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

    private void fulLStop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private short getBeaconColor() {
        telemetry.addData("Color", "RGB: %1$s, %2$s, %3$s", beaconColorSensor.red(), beaconColorSensor.green(), beaconColorSensor.blue());

        if (beaconColorSensor.red() > beaconColorSensor.blue() && beaconColorSensor.red() > beaconColorSensor.green()) {
            DIM.setLED(1, true);           //Red ON
            DIM.setLED(0, false);          //Blue OFF
            telemetry.addData("Beacon", "RED");
            return 1;
        } else if (beaconColorSensor.blue() > beaconColorSensor.red() && beaconColorSensor.blue() > beaconColorSensor.green()) {
            DIM.setLED(1, false);          //Red OFF
            DIM.setLED(0, true);           //Blue ON
            telemetry.addData("Beacon", "BLUE");
            return 2;
        } else {
            DIM.setLED(1, false);           //Red OFF
            DIM.setLED(0, false);           //Blue OFF
            telemetry.addData("Beacon", "Not Detected");
            return 0;
        }
    }

    private void tankDrive(double left, double right) {
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);
    }

    private void lineFollow(boolean onRightSide) {
        double error = lineLightSensor.getLightDetected() - setPoint;
        integral += error;
        double derivative = error - preError;
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative) * (onRightSide ? 1 : -1);

        double leftOut = Range.clip(Tp + output, -0.5, 0.5);
        double rightOut = Range.clip(Tp - output, -0.5, 0.5);

        telemetry.addData("Motor output", "%1$s, %2$s", leftOut, rightOut);
        tankDrive(leftOut, rightOut);
        preError = error;
    }

    private double wallFollow(double distSetPoint) {
        double dist = getDistance();
        double distError = dist - distSetPoint;

        telemetry.addData("Distance", dist);

        distIntegral += distError;
        double distDerivative = distError - distPreError;

        distPreError = distError;

        return (distKp * distError) + (distKi * distIntegral) + (distKd * distDerivative);
    }

    private double getDistance() {
        double distIR = 27.86 * Math.pow(sharpIR.getVoltage(), -1.15);
        double distUS = ultrasonicSensor.getVoltage() / (ultrasonicSensor.getMaxVoltage() / 1024);
        return distIR < 23 ? distIR : distUS;
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