package org.firstinspires.ftc.teamcode.WorldRobotPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "New TeleOp")
public class WorldsRobotTeleOp extends OpMode {
    private WorldRobotDevices robot = new WorldRobotDevices();

    private boolean lastLT = false;
    private boolean lastY1 = false;
    private boolean lastY2 = false;
    private boolean collectionOn = false;
    private boolean liftOffsetEnabled = false;
    private boolean holdCapBall = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        //Drive
        double left = gamepad1.left_stick_y - gamepad1.right_stick_x;
        double right = gamepad1.left_stick_y + gamepad1.right_stick_x;

        robot.leftFrontMotor.setPower(left);
        robot.leftBackMotor.setPower(left);
        robot.rightFrontMotor.setPower(right);
        robot.rightBackMotor.setPower(right);

        //Collection
        boolean currentLT = gamepad1.left_trigger > 0.5;
        if (!lastLT && currentLT) {
            collectionOn = !collectionOn;
        }
        robot.collectionMotor.setPower((collectionOn ? 1 : 0)-gamepad1.right_trigger);

        //Flicker
        robot.flickerMotor.setPower(gamepad2.left_trigger-gamepad2.right_trigger);

        robot.ballIndexer.setPosition(gamepad2.left_bumper ? robot.BIS_LOAD : (gamepad2.right_bumper ? robot.BIS_REVERSE : robot.BIS_IDLE));

        //Button pushers
        if (gamepad2.dpad_left) {
            robot.leftButtonPushers.setPosition(robot.LBPS_LEFT);
            robot.rightButtonPushers.setPosition(robot.RBPS_LEFT);
        } else if (gamepad2.dpad_right) {
            robot.leftButtonPushers.setPosition(robot.RBPS_RIGHT);
            robot.rightButtonPushers.setPosition(robot.RBPS_RIGHT);
        } else {
            robot.leftButtonPushers.setPosition(robot.LBPS_CENTER);
            robot.rightButtonPushers.setPosition(robot.RBPS_CENTER);
        }

        robot.frontPusher.setPosition(gamepad2.b ? robot.FPS_RELEASE : robot.FPS_HOLD);

        //Lift
        int liftPosMain = (robot.liftLeft.getCurrentPosition() + robot.liftRight.getCurrentPosition()) / 2;
        boolean currentY2 = gamepad2.y;
        telemetry.addData("Lift value", liftPosMain);

        if (currentY2 && !lastY2) {
            liftOffsetEnabled = !liftOffsetEnabled;
        }

        double liftMainSpeed = liftPosMain >= 6480 ? 0 : 1;

        robot.liftLeft.setPower(((gamepad2.dpad_up ? liftMainSpeed : 0) - (gamepad2.dpad_down ? 0.5 : 0)) - (liftOffsetEnabled ? gamepad2.left_stick_y : 0));
        robot.liftRight.setPower(((gamepad2.dpad_up ? liftMainSpeed : 0) - (gamepad2.dpad_down ? 0.5 : 0)) - (liftOffsetEnabled ? gamepad2.right_stick_y : 0));

        //Release lift
        robot.liftRelease.setPosition(gamepad2.a ? robot.LRS_RELEASE : robot.LRS_HOLD);

        //Cap ball hold
        boolean currentY1 = gamepad1.y;
        if (currentY1 && !lastY1) {
            holdCapBall = !holdCapBall;
        }

        robot.capHold.setPosition(holdCapBall ? robot.CBHS_GRIP : robot.CBHS_IDLE);

        lastLT = currentLT;
        lastY1 = currentY1;
        lastY2 = currentY2;
    }
}