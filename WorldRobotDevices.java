package org.firstinspires.ftc.teamcode.WorldRobotPrograms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * World robot device initialization class
 */

public class WorldRobotDevices {
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor collectionMotor = null;
    public DcMotor flickerMotor = null;
    public DcMotor liftLeft = null;
    public DcMotor liftRight = null;

    public Servo ballIndexer = null;
    public Servo leftButtonPushers = null;
    public Servo rightButtonPushers = null;
    public Servo frontPusher = null;
    public Servo liftRelease = null;
    public Servo capHold = null;

    //Servo position constants
    public final double BIS_IDLE = 0.5, BIS_LOAD = 0, BIS_REVERSE = 1;
    public final double LBPS_CENTER = 0.580, LBPS_LEFT = 0.795, LBPS_RIGHT = 0.365;
    public final double RBPS_CENTER = 0.485, RBPS_LEFT = 0.270, RBPS_RIGHT = 0.700;
    public final double LRS_HOLD = 0.7508, LRS_RELEASE = 0.3150;
    public final double FPS_HOLD = 0.7214, FPS_RELEASE = 0.5082;
    public final double CBHS_IDLE = 0.8311, CBHS_GRIP = 0;


    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public WorldRobotDevices() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors and Servos
        leftFrontMotor = hwMap.dcMotor.get("LFM");
        rightFrontMotor = hwMap.dcMotor.get("RFM");
        leftBackMotor = hwMap.dcMotor.get("LBM");
        rightBackMotor = hwMap.dcMotor.get("RBM");
        collectionMotor = hwMap.dcMotor.get("CM");
        flickerMotor = hwMap.dcMotor.get("FM");
        liftLeft = hwMap.dcMotor.get("LLM");
        liftRight = hwMap.dcMotor.get("RRM");

        ballIndexer = hwMap.servo.get("BIS");
        leftButtonPushers = hwMap.servo.get("LBPS");
        rightButtonPushers = hwMap.servo.get("RBPS");
        frontPusher = hwMap.servo.get("FPS");
        liftRelease = hwMap.servo.get("LRS");
        capHold = hwMap.servo.get("CBHS");

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        collectionMotor.setDirection(DcMotor.Direction.REVERSE);
        flickerMotor.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRight.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        collectionMotor.setPower(0);
        flickerMotor.setPower(0);
        liftLeft.setPower(0);
        liftRight.setPower(0);

        // Set all servo home positions
        ballIndexer.setPosition(BIS_IDLE);
        leftButtonPushers.setPosition(LBPS_CENTER);
        rightButtonPushers.setPosition(RBPS_CENTER);
        frontPusher.setPosition(FPS_HOLD);
        liftRelease.setPosition(LRS_HOLD);
        capHold.setPosition(CBHS_IDLE);

        // Set all motors to run without encoders
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flickerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
