package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.Globals;

@Config
@TeleOp(name = "Claw Test")
public class OuttakeTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    GamepadEx gamepadEx;
    Timer timer;
    Outtake outtake;

    @Override
    public void runOpMode() {
        timer = new Timer();
        timer.reset();

        gamepadEx = new GamepadEx(gamepad1);
        robot.init(hardwareMap, telemetry);

        outtake = new Outtake();

        waitForStart();

        while (opModeIsActive())
        {
            outtake.update();

            if(gamepad1.right_bumper)
            {
                outtake.setAngle(Outtake.Angle.OUTTAKE);
            }
            else if(gamepad1.left_bumper)
            {
                outtake.setAngle(Outtake.Angle.INTAKE);
            }

            telemetry.update();
        }
    }

}