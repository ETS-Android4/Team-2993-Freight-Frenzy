package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Autonomous (DON'T USE)")
public class RedAuton extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public Hardware robot;
    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
        telemetry.addData("Status:", "Robot Hardware Initialized");
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
        robot.redAuton();
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status:", "Running the Tele-Operation Functions");
        telemetry.update();
    }
}
