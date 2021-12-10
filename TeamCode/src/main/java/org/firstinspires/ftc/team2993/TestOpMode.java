package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Andy TeleOp")
public class TestOpMode extends OpMode {
    double deadZoneX;
    double deadZoneY;
    double deadZoneRotate;
    private DcMotorEx frontRight = null, backRight = null, backLeft = null, frontLeft = null, liftLeft = null, liftRight = null, intake = null, turn = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        frontRight = hardwareMap.get(DcMotorEx.class, "MotorC0");
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight = hardwareMap.get(DcMotorEx.class, "MotorC1");
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft = hardwareMap.get(DcMotorEx.class, "MotorC2");
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft = hardwareMap.get(DcMotorEx.class, "MotorC3");
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        liftLeft = hardwareMap.get(DcMotorEx.class, "MotorE0");
        liftLeft.setDirection(DcMotorEx.Direction.REVERSE);
        liftRight = hardwareMap.get(DcMotorEx.class, "MotorE1");
        liftRight.setDirection(DcMotorEx.Direction.FORWARD);
        intake = hardwareMap.get(DcMotorEx.class, "MotorE2");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        turn = hardwareMap.get(DcMotorEx.class, "MotorE3");
        turn.setDirection(DcMotorEx.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        liftCalc(.30);
        drive(.85);
        intakeCalc(1);
        turnCalc(1);
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    public void drive(double speed) {
        if (Math.abs(gamepad1.left_stick_x) < 0.05) {
            deadZoneX = 0;
        } else {
            deadZoneX = -gamepad1.left_stick_x;
        }
        if (Math.abs(gamepad1.right_stick_y) < 0.05) {
            deadZoneY = 0;
        } else {
            deadZoneY = gamepad1.right_stick_y;
        }
        final double v1 = deadZoneX + deadZoneY;
        final double v2 = deadZoneX - deadZoneY;
        frontRight.setPower(v1 * speed);
        frontLeft.setPower(v2 * speed);
        backRight.setPower(v1 * speed);
        backLeft.setPower(v2 * speed);
    }

    public void driveCalc(double speed) {

        if (Math.abs(gamepad1.left_stick_x) < 0.05) {
            deadZoneX = 0;
        } else {
            deadZoneX = -gamepad1.left_stick_x;
        }
        if (Math.abs(gamepad1.left_stick_y) < 0.05) {
            deadZoneY = 0;
        } else {
            deadZoneY = gamepad1.left_stick_y;
        }
        if (Math.abs(gamepad1.right_stick_x) < 0.05) {
            deadZoneRotate = 0;
        } else {
            deadZoneRotate = gamepad1.right_stick_x;
        }
        double r = Math.hypot(deadZoneX, -deadZoneY);
        double robotAngle = Math.atan2(-deadZoneY, deadZoneX) - Math.PI / 4;
        double rightX = deadZoneRotate / 1.25;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        frontRight.setPower(v1 * speed);
        frontLeft.setPower(v2 * speed);
        backRight.setPower(v3 * speed);
        backLeft.setPower(v4 * speed);
    }

    public void liftCalc(double speed) {
        double deadZoneRB;
        double deadZoneLB;
        if (gamepad1.a) {
            deadZoneRB = 1;
            deadZoneLB = 0;
        } else if (gamepad1.y) {
            deadZoneLB = -1;
            deadZoneRB = 0;
        } else {
            deadZoneRB = 0;
            deadZoneLB = 0;
        }
        final double v1 = deadZoneLB + deadZoneRB;
        liftLeft.setPower(v1 * speed);
        liftRight.setPower(v1 * speed);
    }

    public void intakeCalc(double speed) {
        double deadZoneRT;
        double deadZoneLT;
        if (gamepad1.right_trigger < .05) {
            deadZoneRT = gamepad1.right_trigger;
            deadZoneLT = 0;
        } else if (gamepad1.left_trigger < .05) {
            deadZoneLT = -gamepad1.left_trigger;
            deadZoneRT = 0;
        } else {
            deadZoneRT = 0;
            deadZoneLT = 0;
        }
        final double v1 = deadZoneLT + deadZoneRT;
        intake.setPower(v1 * speed);
    }

    public void turnCalc(double speed) {
        double deadZoneA;
        double deadZoneX;
        if (gamepad1.a) {
            deadZoneA = 1;
            deadZoneX = 0;
        } else if (gamepad1.y) {
            deadZoneX = -1;
            deadZoneA = 0;
        } else {
            deadZoneA = 0;
            deadZoneX = 0;
        }
        final double v1 = deadZoneA + deadZoneX;
        turn.setPower(v1 * speed);
    }

    public void strafe(double speed) {
        if (Math.abs(deadZoneX) < .05 && Math.abs(deadZoneY) < .05 && Math.abs(deadZoneRotate) < .05) {
            if (gamepad1.dpad_right) {
                frontRight.setPower(-speed);
                backRight.setPower(speed);
                backLeft.setPower(-speed);
                frontLeft.setPower(speed);
            } else if (gamepad1.dpad_left) {
                frontRight.setPower(speed);
                backRight.setPower(-speed);
                backLeft.setPower(speed);
                frontLeft.setPower(-speed);
            } else if (gamepad1.dpad_down) {
                frontRight.setPower(speed);
                backRight.setPower(speed);
                backLeft.setPower(speed);
                frontLeft.setPower(speed);
            } else if (gamepad1.dpad_up) {
                frontRight.setPower(-speed);
                backRight.setPower(-speed);
                backLeft.setPower(-speed);
                frontLeft.setPower(-speed);
            }
        }
    }
}