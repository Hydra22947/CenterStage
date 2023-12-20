package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@Config
@TeleOp(name = "Elevator Test")
public class ElevatorTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Elevator elevator;
    BetterGamepad gamepadEx;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new BetterGamepad(gamepad1);

        robot.init(hardwareMap, telemetry);

        elevator = new Elevator(gamepad1);
        Drivetrain drivetrain = new Drivetrain(gamepad1, true);
        waitForStart();

        while (opModeIsActive())
        {
            drivetrain.update();

            if(gamepad1.right_stick_y != 0)
            {
                elevator.setUsePID(false);
            }
            else
            {
                elevator.setUsePID(true);
            }

            elevator.updateTest();
            telemetry.update();
        }
    }

}