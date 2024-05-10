package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@Config
@TeleOp(name = "LiftSubsystem Test", group = "A")
public class ElevatorTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    LiftSubsystem elevator;
    BetterGamepad gamepadEx;
    public static double target = 0;

    @Override
    public void runOpMode() {
        gamepadEx = new BetterGamepad(gamepad1);

        robot.init(hardwareMap, telemetry);

        elevator = new LiftSubsystem(gamepad1, true, true);
        elevator.setAuto(false);

        waitForStart();

        while (opModeIsActive())
        {
            gamepadEx.update();

            if(gamepad1.right_stick_y != 0)
            {
                elevator.setUsePID(false);
            }
            else
            {
                elevator.setUsePID(true);
            }

            elevator.setTarget(target);

            elevator.update();

            telemetry.addData("right motor power", elevator.getControllerR().update());
            telemetry.addData("left motor power", elevator.getControllerL().update());
            telemetry.update();
        }
    }

}