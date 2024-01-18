package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@Config
@TeleOp(name = "Elevator Test")
public class ElevatorTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Elevator elevator;
    BetterGamepad gamepadEx;
    public static double target = 0;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new BetterGamepad(gamepad1);

        robot.init(hardwareMap, telemetry);

        elevator = new Elevator(gamepad1);
        elevator.setAuto(false);
        Drivetrain drivetrain = new Drivetrain(gamepad1, true);

        waitForStart();

        while (opModeIsActive())
        {
            gamepadEx.update();
            drivetrain.update();

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

            telemetry.addData("right motor", robot.elevatorMotorRight.getCurrentPosition());
            telemetry.addData("left motor", robot.elevatorMotorLeft.getCurrentPosition());
            telemetry.addData("right motor power", elevator.getController());
            telemetry.addData("left motor power", elevator.getController2());
            telemetry.update();
        }
    }

}