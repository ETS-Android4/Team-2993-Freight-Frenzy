package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Op Mode (Working)")
public class TestOpMode extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    double deadZoneSX;
    double deadZoneSY;
    private DcMotorEx frontRight = null, backRight = null, backLeft = null, frontLeft = null, lift = null, intake = null, turn = null;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        frontRight = hardwareMap.get(DcMotorEx.class, "MotorC0");
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight = hardwareMap.get(DcMotorEx.class, "MotorC1");
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft = hardwareMap.get(DcMotorEx.class, "MotorC2");
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft = hardwareMap.get(DcMotorEx.class, "MotorC3");
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        lift = hardwareMap.get(DcMotorEx.class, "MotorE0");
        lift.setDirection(DcMotorEx.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "MotorE1");
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        turn = hardwareMap.get(DcMotorEx.class, "MotorE2");
        turn.setDirection(DcMotorEx.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        liftOp(.5);
        driveOp(.85);
        intakeOp(.75);
        turnOp(1);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    public void driveOp(double driveSpeed) {
        if (Math.abs(gamepad1.right_stick_x) > .05) {
            deadZoneSY = gamepad1.right_stick_x;
        } else if (Math.abs(gamepad1.left_stick_y) > .05) {
            deadZoneSX = gamepad1.left_stick_y;
        } else {
            deadZoneSX = 0;
            deadZoneSY = 0;
        }
        final double left = deadZoneSX + deadZoneSY;
        final double right = deadZoneSX - deadZoneSY;
        frontLeft.setPower(left * driveSpeed);
        backLeft.setPower(left * driveSpeed);
        frontRight.setPower(right * driveSpeed);
        backRight.setPower(right * driveSpeed);
    }

    public void intakeOp(double speed) {
        double deadZoneRT = 0;
        double deadZoneLT = 0;
        if (gamepad1.right_trigger > .05) {
            deadZoneRT = gamepad1.right_trigger;
            deadZoneLT = 0;
        } else if (gamepad1.left_trigger > .05) {
            deadZoneLT = -gamepad1.left_trigger;
            deadZoneRT = 0;
        }
        final double v1 = deadZoneLT + deadZoneRT;
        intake.setPower(v1 * speed);
    }

    public void turnOp(double speed) {
        double deadZoneA;
        double deadZoneX;
        if (gamepad1.b) {
            deadZoneA = 1;
            deadZoneX = 0;
        } else if (gamepad1.x) {
            deadZoneX = -1;
            deadZoneA = 0;
        } else {
            deadZoneA = 0;
            deadZoneX = 0;
        }
        final double v1 = deadZoneA + deadZoneX;
        turn.setPower(v1 * speed);
    }

    public void liftOp(double speed) {
        double deadZoneA;
        double deadZoneY;
        if (gamepad1.a) {
            deadZoneA = 1;
            deadZoneY = 0;
        } else if (gamepad1.y) {
            deadZoneY = -1;
            deadZoneA = 0;
        } else{
            deadZoneA = 0;
            deadZoneY = 0;
        }
        final double v1 = deadZoneA + deadZoneY;
        lift.setPower(v1 * speed);
    }
}