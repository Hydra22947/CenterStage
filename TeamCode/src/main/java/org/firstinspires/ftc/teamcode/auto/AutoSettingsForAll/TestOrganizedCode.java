package org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.PlacePurpleActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.ClawSide;

@Config
@Autonomous(name = "2+2 - Auto Blue Left MAX Organized Code")
@Disabled
public class TestOrganizedCode extends LinearOpMode {

        private final RobotHardware robot = RobotHardware.getInstance();
        ElapsedTime time;

    // subsystems
    LiftSubsystem elevator;
    IntakeSubsystem intake;
    OuttakeSubsystem outtake;
    Claw claw;
    IntakeExtensionSubsystem intakeExtension;
    AutoConstants autoConstants;


    DepositActions depositActions;
    PlacePurpleActions intakeActions;
    UpdateActions updateActions;

    MaxBlueConstants maxBlueConstants;



        enum PropLocation
        {
            LEFT,
            CENTER,
            RIGHT
        }

        public static PropLocation propLocation = PropLocation.CENTER;

        SequentialAction blueLeftLeft;
        SequentialAction blueLeftMiddle;
        SequentialAction blueLeftRight;

        @Override
        public void runOpMode() {
            time = new ElapsedTime();

            maxBlueConstants = new MaxBlueConstants(elevator, intake , outtake, claw ,intakeExtension, hardwareMap ,telemetry);

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueLeft);

            autoConstants = new AutoConstants();

            elevator = new LiftSubsystem(true);
            outtake = new OuttakeSubsystem();
            claw = new Claw();
            intake = new IntakeSubsystem();
            intakeExtension = new IntakeExtensionSubsystem(true);

            intakeExtension.setAuto(true);
            elevator.setAuto(true);

            depositActions = new DepositActions(elevator, intake, claw, outtake, intakeExtension);
            intakeActions = new PlacePurpleActions(intake, intakeExtension, claw);
            updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

            blueLeftRight = new SequentialAction(
                    maxBlueConstants.placePurplePixel_RIGHT,
                    maxBlueConstants.placeYellowPixel_RIGHT,
                    maxBlueConstants.intake54_RIGHT,
                    maxBlueConstants.goPlaceCyclesRight,
                    maxBlueConstants.goPark

            );

            blueLeftMiddle= new SequentialAction(
                    maxBlueConstants.placePurplePixel_MIDDLE,
                    maxBlueConstants.placeYellowPixel_MIDDLE,
                    maxBlueConstants.intake54_MIDDLE,
                    maxBlueConstants.goPlaceCyclesLeftOrMid,
                    maxBlueConstants.goPark

            );

            blueLeftLeft = new SequentialAction(
                    maxBlueConstants.placePurplePixel_LEFT,
                    maxBlueConstants.placeYellowPixel_LEFT,
                    maxBlueConstants.intake54_LEFT,
                    maxBlueConstants.goPlaceCyclesLeftOrMid,
                    maxBlueConstants.goPark

            );
            while (opModeInInit() && !isStopRequested()) {
                intake.setAngle(IntakeSubsystem.Angle.MID);

                intake.updateState(IntakeSubsystem.ClawState.CLOSE, ClawSide.BOTH);
                claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
                outtake.setAngle(OuttakeSubsystem.Angle.INTAKE);
                telemetry.addLine("Initialized");
                telemetry.update();
            }

            waitForStart();

            if (isStopRequested()) return;

            switch (propLocation)
            {
                case LEFT:
                    runBlocking(new ParallelAction(
                            //trajBlueLeft,
                            updateActions.updateSystems()
                    ));
                    break;
                case CENTER:

                    runBlocking(new ParallelAction(
                            blueLeftRight,
                            updateActions.updateSystems()
                    ));
                    break;
                case RIGHT:
                    runBlocking(new ParallelAction(
                            //trajBlueRight,
                            updateActions.updateSystems()
                    ));
                    break;
            }
        }


    }
