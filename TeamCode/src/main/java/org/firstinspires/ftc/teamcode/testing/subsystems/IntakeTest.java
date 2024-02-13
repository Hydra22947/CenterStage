package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
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
    BetterGamepad gamepadEx;
    Drivetrain drive;
    public static boolean DEBUG = true;

    public static Intake.Angle angles = Intake.Angle.OUTTAKE;

    @Override
    public void runOpMode() {
        gamepadEx = new BetterGamepad(gamepad1);
        robot.init(hardwareMap, telemetry);

        intake = new Intake();
        drive = new Drivetrain(gamepad1, true);

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            intake.update();
            gamepadEx.update();

            intake.move(angles);

            if(gamepadEx.rightBumperOnce())
            {
                intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            }
            else if(gamepadEx.leftBumperOnce())
            {
                intake.updateClawState(Intake.ClawState.OPEN, ClawSide.BOTH);
            }
            else if(gamepadEx.right_trigger == 1)
            {
                intake.updateClawState(Intake.ClawState.INDETERMINATE, ClawSide.BOTH);
            }



            telemetry.update();
        }

    }

}
