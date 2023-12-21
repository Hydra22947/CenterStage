package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@TeleOp(name = "Intake Test")
public class IntakeTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Intake intake;
    IntakeExtension intakeExtension;
    BetterGamepad gamepadEx;
    Drivetrain drive;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new BetterGamepad(gamepad1);
        robot.init(hardwareMap, telemetry);

        intake = new Intake();
        drive = new Drivetrain(gamepad1, true);

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            intake.update();

            if(gamepadEx.rightBumperOnce())
            {
                intake.move(Intake.Angle.INTAKE);
                intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH);
            }
            else if(gamepadEx.leftBumperOnce())
            {
                intake.move(Intake.Angle.TRANSFER);
                intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            }
            else if(gamepadEx.rightBumperOnce() && gamepadEx.right_trigger == 1)
            {
                intakeExtension.openExtension();
                intake.move(Intake.Angle.INTAKE);
                intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH);
            }
            else if(gamepadEx.leftBumperOnce() && gamepadEx.right_trigger != 1)
            {
                intakeExtension.closeExtension();
                intake.move(Intake.Angle.TRANSFER);
                intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            }

            telemetry.update();
        }

    }

}
