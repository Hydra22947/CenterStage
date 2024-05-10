package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtensionSubsystem;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@Config
@TeleOp(name = "IntakeSubsystem Extension Test", group = "A")
public class IntakeExtensionTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    IntakeExtensionSubsystem intakeExtension;
    BetterGamepad gamepadEx;
    public static double target = 0;

    @Override
    public void runOpMode() {
        gamepadEx = new BetterGamepad(gamepad1);

        robot.init(hardwareMap, telemetry);

        intakeExtension = new IntakeExtensionSubsystem(gamepad1, true);

        waitForStart();

        while (opModeIsActive())
        {
            gamepadEx.update();

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