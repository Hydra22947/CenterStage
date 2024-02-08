package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@Config
@TeleOp(name = "Intake Extension Test")
public class IntakeExtensionTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    IntakeExtension intakeExtension;
    BetterGamepad gamepadEx;
    public static double target = 0;

    @Override
    public void runOpMode() {
        gamepadEx = new BetterGamepad(gamepad1);

        robot.init(hardwareMap, telemetry, true);

        intakeExtension = new IntakeExtension(gamepad1, true);
        Drivetrain drivetrain = new Drivetrain(gamepad1, true);

        waitForStart();

        while (opModeIsActive())
        {
            gamepadEx.update();
            drivetrain.update();

            if(gamepad1.right_stick_y != 0)
            {
                intakeExtension.setUsePID(false);
            }
            else
            {
                intakeExtension.setUsePID(true);
            }

            intakeExtension.setTarget(target);

            intakeExtension.update();

            telemetry.addData("motor power", intakeExtension.getController().update());
            telemetry.update();
        }
    }

}