package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutonFunctions {
    public static final int cpr = 1680; //Counts per Revolution//
    public static final double cpi = cpr / (4 * Math.PI); //Counts per Inch//
    Hardware robot;

    public void goForward(double speed, int in) {
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        robot.frontRight.setTargetPosition(target);
        robot.backRight.setTargetPosition(target);
        robot.backLeft.setTargetPosition(target);
        robot.frontLeft.setTargetPosition(target);
        robot.frontRight.setPower(speed);
        robot.backRight.setPower(speed);
        robot.backLeft.setPower(speed);
        robot.frontLeft.setPower(speed);
        while (robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.frontLeft.isBusy() && robot.backLeft.isBusy()) {
            robot.distanceLeft.getDistance(DistanceUnit.INCH);
            robot.distanceRight.getDistance(DistanceUnit.INCH);
        }
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void goBackward(double speed, int in) {
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        robot.frontRight.setTargetPosition(target);
        robot.backRight.setTargetPosition(target);
        robot.backLeft.setTargetPosition(target);
        robot.frontLeft.setTargetPosition(target);
        robot.frontRight.setPower(-speed);
        robot.backRight.setPower(-speed);
        robot.backLeft.setPower(-speed);
        robot.frontLeft.setPower(-speed);
        while (robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.frontLeft.isBusy() && robot.backLeft.isBusy()) {
            robot.distanceLeft.getDistance(DistanceUnit.INCH);
            robot.distanceRight.getDistance(DistanceUnit.INCH);
        }
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void strafeLeft(double speed, int in) {
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        robot.frontRight.setTargetPosition(target);
        robot.backRight.setTargetPosition(target);
        robot.backLeft.setTargetPosition(target);
        robot.frontLeft.setTargetPosition(target);
        robot.frontRight.setPower(speed);
        robot.backRight.setPower(-speed);
        robot.backLeft.setPower(speed);
        robot.frontLeft.setPower(-speed);
        while (robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.frontLeft.isBusy() && robot.backLeft.isBusy()) {
            robot.distanceLeft.getDistance(DistanceUnit.INCH);
            robot.distanceRight.getDistance(DistanceUnit.INCH);
        }
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void strafeRight(double speed, int in) {
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        robot.frontRight.setTargetPosition(target);
        robot.backRight.setTargetPosition(target);
        robot.backLeft.setTargetPosition(target);
        robot.frontLeft.setTargetPosition(target);
        robot.frontRight.setPower(-speed);
        robot.backRight.setPower(speed);
        robot.backLeft.setPower(-speed);
        robot.frontLeft.setPower(speed);
        while (robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.frontLeft.isBusy() && robot.backLeft.isBusy()) {
            robot.distanceLeft.getDistance(DistanceUnit.INCH);
            robot.distanceRight.getDistance(DistanceUnit.INCH);
        }
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnLeft(double speed, int in) {
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        robot.frontRight.setTargetPosition(target);
        robot.backRight.setTargetPosition(target);
        robot.backLeft.setTargetPosition(target);
        robot.frontLeft.setTargetPosition(target);
        double angleNow = robot.getHeading() + 90;
        while (robot.getHeading() < (angleNow)) {
            robot.frontRight.setPower(speed);
            robot.backRight.setPower(speed);
            robot.backLeft.setPower(-speed);
            robot.frontLeft.setPower(-speed);
        }
        while (robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.frontLeft.isBusy() && robot.backLeft.isBusy()) {
            robot.distanceLeft.getDistance(DistanceUnit.INCH);
            robot.distanceRight.getDistance(DistanceUnit.INCH);
        }
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnRight(double speed, int in) {
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (in * cpi);
        robot.frontRight.setTargetPosition(target);
        robot.backRight.setTargetPosition(target);
        robot.backLeft.setTargetPosition(target);
        robot.frontLeft.setTargetPosition(target);
        double angleNow = robot.getHeading() + 90;
        while (robot.getHeading() < angleNow) {
            robot.frontRight.setPower(-speed);
            robot.backRight.setPower(-speed);
            robot.backLeft.setPower(speed);
            robot.frontLeft.setPower(speed);
        }
        while (robot.frontRight.isBusy() && robot.backRight.isBusy() && robot.frontLeft.isBusy() && robot.backLeft.isBusy()) {
            robot.distanceLeft.getDistance(DistanceUnit.INCH);
            robot.distanceRight.getDistance(DistanceUnit.INCH);
        }
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopDrive() {
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
}
