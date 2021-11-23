package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drive Op")
public class DriveOp extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Hardware robot;

    @Override
    public void init() {
        telemetry.addData("Status:", "Robot Hardware Initialized");
        robot = new Hardware(hardwareMap);
        robot.right_stick_y = gamepad1.right_stick_y;
        robot.right_stick_x = gamepad1.right_stick_x;
        robot.left_stick_x = gamepad1.left_stick_x;
        robot.right_trigger = gamepad1.right_trigger;
        robot.left_trigger = gamepad1.left_trigger;
        robot.a = gamepad1.a;
        robot.y = gamepad1.y;
        robot.dpad_right = gamepad1.dpad_right;
        robot.dpad_left = gamepad1.dpad_left;
        robot.dpad_up = gamepad1.dpad_up;
        robot.dpad_down = gamepad1.dpad_down;
        telemetry.addData("Status:", "Waiting to Start the Match:");
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
        robot.teleOp(.85, 25, .5, .45);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status:", "Running the Tele-Operation Functions");
        telemetry.update();
    }
}