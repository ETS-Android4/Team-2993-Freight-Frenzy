package org.firstinspires.ftc.team2993;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Hardware {
    public static final int cpr = 1680; //Counts per Revolution//
    public static final double cpi = cpr / (4 * Math.PI); //Counts per Inch//
    public final DcMotorEx frontRight, backRight, backLeft, frontLeft, lift, intake;
    public final Servo dropServo;
    public final TouchSensor liftTouch;
    public final DistanceSensor distanceLeft, distanceRight;
    public final BNO055IMU imu;
    float right_stick_y;
    float left_stick_y;
    float right_stick_x;
    float left_stick_x;
    float right_trigger;
    float left_trigger;
    boolean left_bumper;
    boolean right_bumper;
    boolean a;
    boolean b;
    boolean x;
    boolean y;
    boolean dpad_right;
    boolean dpad_left;
    boolean dpad_up;
    boolean dpad_down;
    double deadZoneX;
    double deadZoneY;
    double deadZoneRotate;
    double deadZoneLift;
    double deadZoneIntake;
    double deadZoneDropServo;

    public Hardware(@NonNull HardwareMap map) {
        frontRight = map.get(DcMotorEx.class, "MotorC0");
        backRight = map.get(DcMotorEx.class, "MotorC1");
        backLeft = map.get(DcMotorEx.class, "MotorC2");
        frontLeft = map.get(DcMotorEx.class, "MotorC3");
        lift = map.get(DcMotorEx.class, "MotorE0");
        intake = map.get(DcMotorEx.class, "MotorE1");
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = map.get(BNO055IMU.class, "IMU");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
        dropServo = map.get(Servo.class, "Servo1");
        dropServo.setDirection(Servo.Direction.FORWARD);
        liftTouch = map.get(TouchSensor.class, "Touch1");
        distanceLeft = map.get(DistanceSensor.class, "Distance1");
        distanceRight = map.get(DistanceSensor.class, "Distance2");
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return angles.firstAngle;
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

    public void intakeCalc(double speed) {
        if (liftTouch.getValue() == 1) {
            deadZoneIntake = 0;
        } else if (liftTouch.getValue() == 0) {
            if (a) {
                deadZoneIntake = speed;
            } else if (y) {
                deadZoneIntake = -speed;
            } else {
                deadZoneIntake = 0;
            }
        }
        intake.setPower(deadZoneIntake);
    }

    public void liftCalc(double speedUp, double speedDown) {
        if (right_trigger >= .05) {
            deadZoneLift = speedUp;
        } else if (left_trigger >= .05) {
            deadZoneY = -speedDown;
        } else {
            deadZoneLift = 0;
        }
        lift.setPower(deadZoneLift);
    }

    public void dropCalc() {
        if (left_bumper) {
            deadZoneDropServo = .5;
        } else if (right_bumper) {
            deadZoneDropServo = 0;
        }
        dropServo.setPosition(deadZoneDropServo);
    }

    public void teleOp() {
        driveCalc(.85);
        strafe(.45);
        intakeCalc(.85);
        liftCalc(.9, .5);
        dropCalc();
    }

    public void goForward(double speed, int in) {
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        frontRight.setTargetPosition(target);
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        frontLeft.setTargetPosition(target);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        backLeft.setPower(speed);
        frontLeft.setPower(speed);
        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) {
            distanceLeft.getDistance(DistanceUnit.INCH);
            distanceRight.getDistance(DistanceUnit.INCH);
        }
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void goBackward(double speed, int in) {
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        frontRight.setTargetPosition(target);
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        frontLeft.setTargetPosition(target);
        frontRight.setPower(-speed);
        backRight.setPower(-speed);
        backLeft.setPower(-speed);
        frontLeft.setPower(-speed);
        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) {
            distanceLeft.getDistance(DistanceUnit.INCH);
            distanceRight.getDistance(DistanceUnit.INCH);
        }
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void strafeLeft(double speed, int in) {
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        frontRight.setTargetPosition(target);
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        frontLeft.setTargetPosition(target);
        frontRight.setPower(speed);
        backRight.setPower(-speed);
        backLeft.setPower(speed);
        frontLeft.setPower(-speed);
        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) {
            distanceLeft.getDistance(DistanceUnit.INCH);
            distanceRight.getDistance(DistanceUnit.INCH);
        }
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void strafeRight(double speed, int in) {
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        frontRight.setTargetPosition(target);
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        frontLeft.setTargetPosition(target);
        frontRight.setPower(-speed);
        backRight.setPower(speed);
        backLeft.setPower(-speed);
        frontLeft.setPower(speed);
        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) {
            distanceLeft.getDistance(DistanceUnit.INCH);
            distanceRight.getDistance(DistanceUnit.INCH);
        }
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnLeft(double speed, int in) {
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        frontRight.setTargetPosition(target);
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        frontLeft.setTargetPosition(target);
        if (getHeading() > (getHeading() + 90)) {
            frontRight.setPower(speed);
            backRight.setPower(speed);
            backLeft.setPower(-speed);
            frontLeft.setPower(-speed);
        }
        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) {
            distanceLeft.getDistance(DistanceUnit.INCH);
            distanceRight.getDistance(DistanceUnit.INCH);
        }
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnRight(double speed, int in) {
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        frontRight.setTargetPosition(target);
        backRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        frontLeft.setTargetPosition(target);
        if (getHeading() < (getHeading() + 90)) {
            frontRight.setPower(-speed);
            backRight.setPower(-speed);
            backLeft.setPower(speed);
            frontLeft.setPower(speed);
        }
        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) {
            distanceLeft.getDistance(DistanceUnit.INCH);
            distanceRight.getDistance(DistanceUnit.INCH);
        }
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        frontLeft.setPower(0);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopDrive() {
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void redAuton() {
        goForward(50, 12);
        strafeLeft(50, 12);
        stopDrive();
    }

    public void blueAuton() {
        goForward(50, 12);
        strafeRight(50, 12);
        stopDrive();
    }
}