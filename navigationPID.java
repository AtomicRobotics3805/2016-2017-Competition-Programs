package org.firstinspires.ftc.teamcode.WorldRobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AdafruitIMU;


public final class navigationPID {
    private AdafruitIMU AdafruitGyro; //Instance of AdafruitIMU
    private double yawAngle;  //Array to store IMU's output

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    private double[][] movementArray;
    private int movementArrayStep = 0;

    //Variables for PID loop
    private double setPoint = 0;
    private double turnKp;
    private double turnKi;
    private double turnKd;

    private double forwardsKp;
    private double forwardsKi;
    private double forwardsKd;

    private double integral = 0;
    private double preError;

    private double encoderTicksPerInch;//= encoderTicksPerRev / (Math.PI * 3); //Variable to convert encoder ticks to inches
    private int encoderPositionReference; //Variable to use as reference for moveForward and moveBackward
    private OpMode opmode;

    private double completedHeading;

    //Constructor
    public navigationPID(double[][] movementCommandArray, OpMode opmode, String configuredIMUname, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, int encoderTicksPerRev, double wheelDiameter) {
        this.opmode = opmode;
        AdafruitGyro = new AdafruitIMU(opmode, configuredIMUname);

        //Assign the DcMotor instances to those of the main class
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.leftFront = leftFront;
        this.leftBack = leftBack;

        movementArray = movementCommandArray;

        encoderTicksPerInch = encoderTicksPerRev / (Math.PI * wheelDiameter);
    }

    //**********Public methods***********

    public void initialize() {
        AdafruitGyro.startIMU(); //Prepare the IMU for I2C communication

        encoderPositionReference = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2; //Set encoderPositionReference to the mean value of the current motor positions
    }

    public void tuneTurnGains(double newKp, double newKi, double newKd) {
        //Set the gains for the PID loop
        turnKp = newKp;
        turnKi = newKi;
        turnKd = newKd;
    }
    public void tuneForwardsGains(double newKp, double newKi, double newKd) {
        //Set the gains for the PID loop
        forwardsKp = newKp;
        forwardsKi = newKi;
        forwardsKd = newKd;
    }

    public void loopNavigation() {
        yawAngle = AdafruitGyro.getYaw();

        opmode.telemetry.addData("Heading, Goal", "%1$s, %2$s", currentHeading(), movementArray[movementArrayStep][2]);
        opmode.telemetry.addData("Completed Heading", completedHeading);
        opmode.telemetry.addData("Encoder values", "\nRF: %1$s LF: %2$s\nRB: %3$s LB: %4$s", rightFront.getCurrentPosition(), leftFront.getCurrentPosition(), rightBack.getCurrentPosition(), leftBack.getCurrentPosition());
        opmode.telemetry.addData("Motor powers", "\nRF: %1$s LF: %2$s\nRB: %3$s LB: %4$s", rightFront.getPower(), leftFront.getPower(), rightBack.getPower(), leftBack.getPower());

        //debugLogFile.write("Heading, Goal", currentHeading() + ", " + movementArray[movementArrayStep][2]);
        //debugLogFile.write("Completed Heading", completedHeading);
        //debugLogFile.write("Encoder values", "RF: " + rightFront.getCurrentPosition() + "LF: " + leftFront.getCurrentPosition() + "RB: " + rightBack.getCurrentPosition() + "LB: " + leftBack.getCurrentPosition());
        //debugLogFile.write("Motor powers", "RF: " + rightFront.getPower() + "LF: " + leftFront.getPower() + "RB: " + rightBack.getPower() + "LB: " + leftBack.getPower());

        move((int) movementArray[movementArrayStep][0], movementArray[movementArrayStep][1], movementArray[movementArrayStep][2]);
    }

    public int navigationStep() {
        return movementArrayStep;
    }

    public int navigationType() {
        return (int) movementArray[movementArrayStep][0];
    }

    public double currentHeading() {
        return yawAngle;
    }

    public void forceNextMovement() {
        nextMovement();
    }

    public void moveNoStop(double TpForwards, double TpSlide) {
        yawAngle = AdafruitGyro.getYaw();
        loopPID(TpForwards, forwardsKp, forwardsKi, forwardsKd);
    }

    //**********Private methods**********

    private void move(int movementType, double Tp, double goalValue) {
        switch (movementType) {
            case 1:
                moveForward(goalValue, Tp);
                break;
            case 2:
                moveBackward(goalValue, Tp);
                break;
            case 3:
                rotateCW(goalValue);
                break;
            case 4:
                rotateCCW(goalValue);
                break;
            case 5:
                fullStop();
                break;
            default:
                break;
        }
    }

    private void moveForward(double goalDistanceInch, double Tp) { //Movement val == 1
        int distanceTraveled = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;

        if (distanceTraveled < convertInchesToEncoderTicks(goalDistanceInch) + encoderPositionReference) {
            loopPID(Tp, forwardsKp, forwardsKi, forwardsKd);
        } else {
            nextMovement();
        }
    }

    private void moveBackward(double goalDistanceInch, double Tp) { //Movement val == 2
        int distanceTraveled = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;

        if (distanceTraveled > convertInchesToEncoderTicks(goalDistanceInch) + encoderPositionReference) {
            loopPID(Tp, forwardsKp, forwardsKi, forwardsKd);
        } else {
            nextMovement();
        }
    }

    private void rotateCW(double goalAngle) { //Movement val == 3
        setPoint = goalAngle;
        if (yawAngle < setPoint) {
            loopPID(0, turnKp, turnKi, turnKd);
        } else {
            nextMovement();
        }
    }

    private void rotateCCW(double goalAngle) { //Movement val == 4
        setPoint = goalAngle;
        if (yawAngle > setPoint) {
            loopPID(0, turnKp, turnKi, turnKd);
        } else {
            nextMovement();
        }
    }

    private double convertInchesToEncoderTicks(double inches) {
        return encoderTicksPerInch * inches;
    }

    private void fullStop() {
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }

    private void nextMovement() {
        fullStop();
        completedHeading = yawAngle;
        encoderPositionReference = (leftFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2;
        movementArrayStep++;
    }

    private void loopPID(double TpForwards, double Kp, double Ki, double Kd) {
        double error = yawAngle - setPoint;
        integral += error;
        double derivative = error - preError;
        double output = Range.clip((Kp * error) + (Ki * integral) + (Kd * derivative), -1, 1);

        rightFront.setPower(TpForwards - output);
        rightBack.setPower(TpForwards - output);
        leftFront.setPower(TpForwards + output);
        leftBack.setPower(TpForwards + output);
        preError = error;
    }
}