package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@TeleOp(name = "IntakeSubsystem Test", group = "A")
public class IntakeTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    IntakeSubsystem intake;
    BetterGamepad gamepadEx;

    public static IntakeSubsystem.Angle angles = IntakeSubsystem.Angle.OUTTAKE;

    @Override
    public void runOpMode() {
        gamepadEx = new BetterGamepad(gamepad1);
        robot.init(hardwareMap, telemetry);

        intake = new IntakeSubsystem();

        waitForStart();

        while (opModeIsActive()) {
            intake.update();
            gamepadEx.update();

            intake.move(angles);

            if(gamepadEx.rightBumperOnce())
            {
                intake.updateState(IntakeSubsystem.ClawState.CLOSE, ClawSide.BOTH);
            }
            else if(gamepadEx.leftBumperOnce())
            {
                intake.updateState(IntakeSubsystem.ClawState.OPEN, ClawSide.BOTH);
            }
            else if(gamepadEx.right_trigger == 1)
            {
                intake.updateState(IntakeSubsystem.ClawState.INDETERMINATE, ClawSide.BOTH);
            }



            telemetry.update();
        }

    }

}
