package org.firstinspires.ftc.teamcode.testing.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@TeleOp(name = "Intake Outtake Test", group = "A")
public class IntakeOuttakeTest extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    Intake intake;
    Claw claw;

    Outtake outtake;

    public static Outtake.Angle outtakeAngle = Outtake.Angle.INTAKE;

    public static Intake.Angle intakeAngle = Intake.Angle.OUTTAKE;

    public static Claw.ClawState outtakeClaw = Claw.ClawState.OPEN;
    public static Intake.ClawState intakeClaw = Intake.ClawState.OPEN;
    public static boolean useOuttake = true;
    public static boolean useIntake = true;
    public static boolean useIntakeClaw = true;
    public static boolean useOuttakeClaw = true;



    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        intake = new Intake();
        outtake = new Outtake();
        claw = new Claw();

        waitForStart();

        while (opModeIsActive()) {

            if(useIntake)
            {
                intake.move(intakeAngle);
            }

            if(useOuttake)
            {
                outtake.setAngle(outtakeAngle);
            }
            if(useOuttakeClaw)
            {
                claw.setBothClaw(outtakeClaw);
            }

            if(useIntakeClaw)
            {
                intake.updateClawState(intakeClaw, ClawSide.BOTH);
            }

            telemetry.update();
        }

    }

}
