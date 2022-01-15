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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware {
    //Motor initialization - Separate from .xml configuration and is used later on to reference the configuration, "DCMotorEx adds" extra motor conventions over "DCMotorSimple" or "DCMotor"//
    public final DcMotorEx frontRight, backRight, backLeft, frontLeft, lift, intake, turner;
    //Distance Sensor initialization - Separate from .xml configuration and is used later on to reference the configuration//
    private final DistanceSensor distanceLeft, distanceRight;
    //Touch Sensor initialization - Separate from .xml configuration and is used later on to reference the configuration, "TouchSensor" outputs boolean compared to "DigitalChannel"//
    private final TouchSensor liftTouch;
    //IMU Sensor initialization - Separate from .xml configuration and is used later on to reference the configuration//
    private final BNO055IMU imu;

    //Counts per 1 motor encoder revolution//
    public static final int cpr = 1680;
    //Counts per inch of linear travel - cpi = cpr /("wheel diameter" * Math.PI //
    public static final double cpi = cpr / (4 * Math.PI);

    //Variable Setup - Initialize any needed variables here//
    String status = null;

    //Controller Setup = is needed if the drive functions are not in the main file (TeleOp)//
    float right_stick_y, left_stick_y, right_stick_x, left_stick_x, right_trigger, left_trigger;
    boolean left_bumper, right_bumper, a, b, x, y, dpad_right, dpad_left, dpad_up, dpad_down;

    //Hardware Map - Associates the physical hardware devices (assigned in the configuration) to the software definitions (DCMotorEx, DigitalChannel, etc.) without referencing each port and IO Mapping again//
    public Hardware(@NonNull HardwareMap map) {
        frontRight = map.get(DcMotorEx.class, "MotorC0");
        backRight = map.get(DcMotorEx.class, "MotorC1");
        backLeft = map.get(DcMotorEx.class, "MotorC2");
        frontLeft = map.get(DcMotorEx.class, "MotorC3");
        lift = map.get(DcMotorEx.class, "MotorE0");
        intake = map.get(DcMotorEx.class, "MotorE1");
        turner = map.get(DcMotorEx.class, "MotorE2");
        distanceLeft = map.get(DistanceSensor.class, "Distance1");
        distanceRight = map.get(DistanceSensor.class, "Distance2");
        liftTouch = map.get(TouchSensor.class, "Touch1");
        imu = map.get(BNO055IMU.class, "IMU");
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        turner.setDirection(DcMotorEx.Direction.FORWARD);
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
    }

    public double getHeading(AngleUnit angleUnit) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, angleUnit);
        return angles.firstAngle;
    }

    //Custom Functions//

    //Mecchanum Drive Calculations//
    public void mecchanumDriveCalc(double speed) {
        double deadZoneX;
        double deadZoneY;
        double deadZoneRotate;
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

    //Basic 4WD Calculations//
    public void normalDriveOp(double driveSpeed) {
        double deadZoneX = 0;
        double deadZoneY = 0;
        if (Math.abs(right_stick_x) > .05) {
            deadZoneY = right_stick_x;
        } else if (Math.abs(left_stick_y) > .05) {
            deadZoneX = left_stick_y;
        } else {
            deadZoneX = 0;
            deadZoneY = 0;
        }
        final double left = deadZoneX + deadZoneY;
        final double right = deadZoneX - deadZoneY;
        frontLeft.setPower(left * driveSpeed);
        backLeft.setPower(left * driveSpeed);
        frontRight.setPower(right * driveSpeed);
        backRight.setPower(right * driveSpeed);
    }

    //Carousel Turner Op//
    public void turnOp(double speed) {
        double deadZoneA;
        double deadZoneX;
        if (b) {
            deadZoneA = 1;
            deadZoneX = 0;
        } else if (x) {
            deadZoneX = -1;
            deadZoneA = 0;
        } else {
            deadZoneA = 0;
            deadZoneX = 0;
        }
        final double v1 = deadZoneA + deadZoneX;
        turner.setPower(v1 * speed);
    }

    //Intake Op//
    public void intakeOp(double speed) {
        double deadZoneIntake;
        if (a) {
            deadZoneIntake = speed;
        } else if (y) {
            deadZoneIntake = -speed;
        } else {
            deadZoneIntake = 0;
        }
        intake.setPower(deadZoneIntake);
    }

    //Lift Op//
    public void liftOp(double speed) {
        double deadZoneRT = 0;
        double deadZoneLT = 0;
        if (right_trigger > .05) {
            deadZoneRT = right_trigger;
            deadZoneLT = 0;
        } else if (left_trigger > .05) {
            deadZoneLT = -left_trigger;
            deadZoneRT = 0;
        }
        final double v1 = deadZoneLT + deadZoneRT;
        intake.setPower(v1 * speed);
    }

    //Autonomous Functions using Encoders (Self Explanatory)//
    public void goForward(double speed, int in) {
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
    }

    public void goBackward(double speed, int in) {
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
    }

    public void strafeLeft(double speed, int in) {
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
    }

    public void strafeRight(double speed, int in) {
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
    }

    public void turnLeft(double speed, int in) {
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
    }

    public void turnRight(double speed, int in) {
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
    }

    public void reset() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        intake.setPower(0);
        turner.setPower(0);
        lift.setPower(0);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turner.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
}