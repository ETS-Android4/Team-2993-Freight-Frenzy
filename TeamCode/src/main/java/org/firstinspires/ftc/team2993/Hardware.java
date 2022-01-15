package org.firstinspires.ftc.team2993;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
public class Hardware {
    public static final int cpr = 1680; //Counts per Revolution//
    public static final double cpi = cpr / (4 * Math.PI); //Counts per Inch//
    public final DcMotorEx frontRight, backRight, backLeft, frontLeft, lift, intake, turner;
    private DistanceSensor distanceLeft, distanceRight;
    private TouchSensor liftTouch;
    private BNO055IMU imu;
    float right_stick_y, left_stick_y, right_stick_x, left_stick_x, right_trigger, left_trigger;
    boolean left_bumper, right_bumper, a, b, x, y, dpad_right, dpad_left, dpad_up, dpad_down;
    double deadZoneX, deadZoneY, deadZoneRotate, deadZoneIntake, deadZoneLift;
    public Hardware(@NonNull HardwareMap map) {
        frontRight = map.get(DcMotorEx.class, "MotorC0");
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight = map.get(DcMotorEx.class, "MotorC1");
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft = map.get(DcMotorEx.class, "MotorC2");
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft = map.get(DcMotorEx.class, "MotorC3");
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        lift = map.get(DcMotorEx.class, "MotorE0");
        lift.setDirection(DcMotorEx.Direction.FORWARD);
        intake = map.get(DcMotorEx.class, "MotorE1");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        turner = map.get(DcMotorEx.class, "MotorE2");
        turner.setDirection(DcMotorEx.Direction.FORWARD);
        distanceLeft = map.get(DistanceSensor.class, "Distance1");
        distanceRight = map.get(DistanceSensor.class, "Distance2");
        liftTouch = map.get(TouchSensor.class, "Touch1");
        imu = map.get(BNO055IMU.class, "IMU");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
    }

    public double getHeading(AngleUnit angleUnit) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, angleUnit);
        return angles.firstAngle;
    }

    public void mecchanumDriveCalc(double speed) {

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

    public void drive(double driveSpeed) {
        if (Math.abs(right_stick_x) > .05) {
            deadZoneX = right_stick_x;
        } else if(Math.abs(left_stick_y) > .05){
            deadZoneY = left_stick_y;
        } else{
            deadZoneX = 0;
            deadZoneY = 0;
        }
        final double left = deadZoneX + deadZoneY;
        final double right = deadZoneX - deadZoneY;
        frontLeft.setPower(left*driveSpeed);
        backLeft.setPower(left*driveSpeed);
        frontRight.setPower(right*driveSpeed);
        backRight.setPower(right*driveSpeed);
    }
    public void intakeCalc(double speed) {
            if (a) {
                deadZoneIntake = speed;
            } else if (y) {
                deadZoneIntake = -speed;
            } else {
                deadZoneIntake = 0;
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

    public void redAuton() {
    }

    public void blueAuton() {
    }
}