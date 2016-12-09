package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp")
public class MecanumDriveCollection extends OpMode {
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor launcher;
    DcMotor liftLeft;
    DcMotor liftRight;
    DcMotor collectionMotor;

    Servo leftButtonServo;
    Servo rightButtonServo;
    Servo particleServo;
    Servo forkReleaseServo;
    Servo collectionReleaseServo;

    boolean lastX;
    boolean slowMode;
    double scaleFactor = 1;

    boolean lastY;
    boolean liftOffsetEnabled;


    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("RF");
        rightBack = hardwareMap.dcMotor.get("RB");
        leftFront = hardwareMap.dcMotor.get("LF");
        leftBack = hardwareMap.dcMotor.get("LB");
        launcher = hardwareMap.dcMotor.get("SM");
        liftLeft = hardwareMap.dcMotor.get("RLM");
        liftRight = hardwareMap.dcMotor.get("LLM");
        collectionMotor = hardwareMap.dcMotor.get("CM");

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftButtonServo = hardwareMap.servo.get("LBS");
        rightButtonServo = hardwareMap.servo.get("RBS");

        particleServo = hardwareMap.servo.get("PS");
        forkReleaseServo = hardwareMap.servo.get("FRS");
        collectionReleaseServo = hardwareMap.servo.get("CRS");

        leftButtonServo.setDirection(Servo.Direction.REVERSE);

        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //Drive
        if (gamepad1.x && !lastX) {
            slowMode = !slowMode;
        }

        if (slowMode) {
            scaleFactor = 0.25;
        } else {
            scaleFactor = 1;
        }
        double Y1 = gamepad1.left_stick_y * scaleFactor; //Forwards/Backwards
        double X1 = -gamepad1.left_stick_x * scaleFactor; //Left/Right
        double X2 = gamepad1.right_stick_x * scaleFactor; //Rotate

        rightFront.setPower(Y1 - X2 - X1);
        rightBack.setPower(Y1 - X2 + X1);
        leftFront.setPower(Y1 + X2 + X1);
        leftBack.setPower(Y1 + X2 - X1);

        //Button Servos
        if (gamepad2.left_trigger > 0.5) {
            leftButtonServo.setPosition(1);
        } else {
            leftButtonServo.setPosition(0);
        }

        if (gamepad2.right_trigger > 0.5) {
            rightButtonServo.setPosition(1);
        } else {
            rightButtonServo.setPosition(0);
        }

        //Launcher
        launcher.setPower(gamepad2.left_bumper ? 1 : gamepad2.right_bumper ? -0.2 : 0);

        //Lift
        int liftPosMain = (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / 2;
        telemetry.addData("Lift value", liftPosMain);

        if (gamepad2.y && !lastY) {
            liftOffsetEnabled = !liftOffsetEnabled;
        }

        double liftMainSpeed = liftPosMain >= 6480 ? 0 : 0.5;

        liftLeft.setPower(((gamepad2.dpad_up ? liftMainSpeed : 0) - (gamepad2.dpad_down ? 0.5 : 0)) - (liftOffsetEnabled ? gamepad2.left_stick_y : 0));
        liftRight.setPower(((gamepad2.dpad_up ? liftMainSpeed : 0) - (gamepad2.dpad_down ? 0.5 : 0)) - (liftOffsetEnabled ? gamepad2.right_stick_y : 0));

        //Forklift release
        forkReleaseServo.setPosition(gamepad2.a ? 1 : 0);

        //Extra particle
        particleServo.setPosition(gamepad2.b ? 0.9 : 0);

        //Collection release
        collectionReleaseServo.setPosition(gamepad1.b ? 0.5 : 0);

        //Collection
        collectionMotor.setPower((gamepad1.left_trigger - gamepad1.right_trigger) * 0.25);

        lastX = gamepad1.x;
        lastY = gamepad2.y;
    }
}