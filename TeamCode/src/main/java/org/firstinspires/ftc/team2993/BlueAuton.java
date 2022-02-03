package org.firstinspires.ftc.team2993;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Autonomous")
public class BlueAuton extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    public Hardware robot;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        robot = new Hardware(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        robot.reset();
        robot.goForward(1, 45);
        robot.reset();
        robot.turnRight(1, 7);
        robot.reset();
        robot.goForward(1, 5);
        robot.reset();
        robot.intake(-1,9);

    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status:", "Running the Tele-Operation Functions");
        telemetry.update();
    }
}
