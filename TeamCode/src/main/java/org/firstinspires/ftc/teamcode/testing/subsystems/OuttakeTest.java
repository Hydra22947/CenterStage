package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Config
@TeleOp(name = "Claw Test")
@Disabled
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