package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Config
@TeleOp(name = "Intake Test")
public class IntakeTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Intake intake;
    //IntakeExtension intakeExtension;
    GamepadEx gamepadEx;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.init(hardwareMap, telemetry);

        intake = new Intake();

        waitForStart();

        while (opModeIsActive()) {
            intake.update();
            if(gamepad1.right_trigger != 0)
            {
                intake.move(Intake.Angle.INTAKE);
                intake.setManual(true);
                intake.intakeMove(gamepad1.right_trigger);
            }
            else if(gamepad1.left_trigger != 0)
            {
                intake.move(Intake.Angle.TRANSFER);
                intake.setManual(true);
                intake.intakeMove(-gamepad1.left_trigger);
            }
            else
            {
                intake.move(Intake.Angle.TRANSFER);
                intake.setManual(false);
            }

            telemetry.update();
        }

    }

}