package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@TeleOp(name = "Outtake Test")
public class OuttakeTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    BetterGamepad gamepadEx;
    Outtake outtake;
    Claw claw;
    Drivetrain drivetrain;

    public static Outtake.Angle angle = Outtake.Angle.INTAKE;
    public static boolean DEBUG = true;


    @Override
    public void run() {
        outtake.update();
        drivetrain.update();

        if(!DEBUG)
        {
            if(gamepad1.right_bumper)
            {
                claw.updateState(Claw.ClawState.CLOSED, ClawSide.BOTH);
                outtake.setAngle(Outtake.Angle.OUTTAKE);
            }
            else if(gamepad1.left_bumper)
            {
                outtake.setAngle(Outtake.Angle.INTAKE);
                claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            }
        }
        else
        {
            outtake.setAngle(angle);
        }

        telemetry.update();
    }

    @Override
    public void initialize() {
        gamepadEx = new BetterGamepad(gamepad1);
        robot.init(hardwareMap, telemetry);

        drivetrain = new Drivetrain(gamepad1, true);
        outtake = new Outtake();
        claw = new Claw(this);
    }

}