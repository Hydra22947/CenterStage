package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@TeleOp(name = "OuttakeSubsystem Test", group = "A")
public class OuttakeTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    BetterGamepad gamepadEx;
    OuttakeSubsystem outtake;

    public static OuttakeSubsystem.Angle angle = OuttakeSubsystem.Angle.INTAKE;
    public static boolean DEBUG = true;


    @Override
    public void run() {
        outtake.update();

        if(!DEBUG)
        {
            if(gamepad1.right_bumper)
            {
                outtake.updateState(OuttakeSubsystem.ClawState.CLOSED, ClawSide.BOTH);
                outtake.setAngle(OuttakeSubsystem.Angle.OUTTAKE);
            }
            else if(gamepad1.left_bumper)
            {
                outtake.setAngle(OuttakeSubsystem.Angle.INTAKE);
                outtake.updateState(OuttakeSubsystem.ClawState.OPEN, ClawSide.BOTH);
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

        outtake = new OuttakeSubsystem();
    }

}