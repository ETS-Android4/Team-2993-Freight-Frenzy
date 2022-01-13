package org.firstinspires.ftc.team2993;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
public class Hardware {
    public static final int cpr = 1680; //Counts per Revolution//
    public static final double cpi = cpr / (4 * Math.PI); //Counts per Inch//
    public final DcMotorEx frontRight, backRight, backLeft, frontLeft, lift, intake, turner;
    private DistanceSensor distanceLeft, distanceRight;
    private TouchSensor liftTouch;
    private BNO055IMU imu;
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
    double deadZoneIntake;
    double deadZoneLift;
    String status;

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
           status = "Forwards";
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
            status = "Backwards";
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
            status = "Strafing Left";
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
            status = "Strafing Right";
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
        frontRight.setPower(-speed);
        backRight.setPower(speed);
        backLeft.setPower(-speed);
        frontLeft.setPower(speed);
        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) {
            status = "Turning Left";
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
        frontRight.setPower(-speed);
        backRight.setPower(speed);
        backLeft.setPower(-speed);
        frontLeft.setPower(speed);
        while (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy()) {
            status = "Strafing Right";
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

    public void stopResetDrive() {
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
        stopResetDrive();
    }

    public void blueAuton() {
        goForward(50, 12);
        stopResetDrive();
    }
}