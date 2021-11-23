package org.firstinspires.ftc.team2993;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {
    private final DcMotorEx frontRight, backRight, backLeft, frontLeft, liftLeft, liftRight, intake;
    float right_stick_y;
    float left_stick_y;
    float right_stick_x;
    float left_stick_x;
    float right_trigger;
    float left_trigger;
    boolean a;
    boolean y;
    boolean dpad_right;
    boolean dpad_left;
    boolean dpad_up;
    boolean dpad_down;
    double deadZoneX;
    double deadZoneY;
    double deadZoneRotate;
    public Hardware(@NonNull HardwareMap map) {
        frontRight = map.get(DcMotorEx.class, "MotorC0");
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight = map.get(DcMotorEx.class, "MotorC1");
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft = map.get(DcMotorEx.class, "MotorC2");
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft = map.get(DcMotorEx.class, "MotorC3");
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        liftLeft = map.get(DcMotorEx.class, "MotorE0");
        liftLeft.setDirection(DcMotorEx.Direction.REVERSE);
        liftRight = map.get(DcMotorEx.class, "MotorE1");
        liftRight.setDirection(DcMotorEx.Direction.FORWARD);
        intake = map.get(DcMotorEx.class, "MotorE2");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void driveCalc(double speed) {

        if (Math.abs(left_stick_x) < 0.05) {
            deadZoneX = 0;
        } else {
            deadZoneX = -left_stick_x;
        }
        if (Math.abs(left_stick_y) < 0.05) {
            deadZoneY = 0;
        } else {
            deadZoneY = left_stick_y;
        }
        if (Math.abs(right_stick_x) < 0.05) {
            deadZoneRotate = 0;
        } else {
            deadZoneRotate = right_stick_x;
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
        if (a) {
            deadZoneRB = 1;
            deadZoneLB = 0;
        } else {
            deadZoneRB = 0;
        }
        if (y) {
            deadZoneLB = -1;
            deadZoneRB = 0;
        } else {
            deadZoneLB = 0;
        }
        final double v1 = deadZoneRB + deadZoneRB;
        liftLeft.setPower(v1 * speed);
        liftRight.setPower(v1 * speed);
    }

    public void intakeCalc(double speed) {
        double deadZoneRT;
        double deadZoneLT;
        if (right_trigger < .05) {
            deadZoneRT = right_trigger;
            deadZoneLT = 0;
        } else {
            deadZoneRT = right_trigger;
        }
        if (left_trigger < .05) {
            deadZoneLT = 0;
        } else {
            deadZoneLT = left_trigger;
            deadZoneRT = 0;
        }
        final double v1 = deadZoneLT + deadZoneRT;
        intake.setPower(v1 * speed);
    }

    public void strafe(double speed) {
        if (Math.abs(deadZoneX) < .05 && Math.abs(deadZoneY) < .05 && Math.abs(deadZoneRotate) < .05) {
            if (dpad_right) {
                frontRight.setPower(-speed);
                backRight.setPower(speed);
                backLeft.setPower(-speed);
                frontLeft.setPower(speed);
            } else if (dpad_left) {
                frontRight.setPower(speed);
                backRight.setPower(-speed);
                backLeft.setPower(speed);
                frontLeft.setPower(-speed);
            } else if (dpad_down) {
                frontRight.setPower(speed);
                backRight.setPower(speed);
                backLeft.setPower(speed);
                frontLeft.setPower(speed);
            } else if (dpad_up) {
                frontRight.setPower(-speed);
                backRight.setPower(-speed);
                backLeft.setPower(-speed);
                frontLeft.setPower(-speed);
            }
        }
    }

    public void teleOp(double driveSpeed, double liftSpeed, double intakeSpeed, double strafeSpeed) {
        driveCalc(driveSpeed);
        liftCalc(liftSpeed);
        intakeCalc(intakeSpeed);
        strafe(strafeSpeed);
    }

    public void DriveStraightInches(double pow, int in) {
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        int target = (int) (in);// * CPI);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setTargetPosition(target);
        frontLeft.setPower(pow);
        frontRight.setPower(pow);
        backLeft.setPower(pow);
        backRight.setPower(pow);
        if (!frontLeft.isBusy() && !frontRight.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}