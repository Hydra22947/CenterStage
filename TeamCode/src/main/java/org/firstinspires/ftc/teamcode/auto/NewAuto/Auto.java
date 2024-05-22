package org.firstinspires.ftc.teamcode.auto.NewAuto;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings.cycleVision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.auto.Actions.DepositActions;
import org.firstinspires.ftc.teamcode.auto.Actions.IntakeActions;
import org.firstinspires.ftc.teamcode.auto.Actions.UpdateActions;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.AutoSettingsForAll.AutoSettings;
import org.firstinspires.ftc.teamcode.auto.FailSafe.PositionLocker;
import org.firstinspires.ftc.teamcode.auto.NewAuto.AutoBlue.AutoBlueLeftRight;
import org.firstinspires.ftc.teamcode.auto.NewAuto.AutoBlue.AutoBlueMidRight;
import org.firstinspires.ftc.teamcode.auto.NewAuto.AutoBlue.AutoBlueRightRight;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.testing.vision.PropPipelineBlueRight;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "2+3 - Auto Right Blue")
public class Auto extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    ElapsedTime time;

    // subsystems
    Elevator elevator;
    Intake intake;
    Outtake outtake;
    Claw claw;
    IntakeExtension intakeExtension;
    AutoConstants autoConstants;


    DepositActions depositActions;
    IntakeActions intakeActions;
    UpdateActions updateActions;
    PositionLocker poseLocker;
    PositionLocker.LockPositionAction poseLockerAction;
    public static AutoSettings.PropLocation propLocation = AutoSettings.PropLocation.MIDDLE;

    PropPipelineBlueRight propPipelineBlueRight;
    OpenCvWebcam webcam;

    SequentialAction readyForDepositAction;
    int tempHeight = 1250;
    int tempHeightMin = 1050;
    int tempHeightMax = 1250;

    boolean first = true;
    SubsystemActions subsystemActions;

    void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

    }

    @Override
    public void runOpMode() throws InterruptedException {
        BetterGamepad betterGamepad2 = new BetterGamepad(gamepad2);
        time = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        propPipelineBlueRight = new PropPipelineBlueRight();
        robot.init(hardwareMap, telemetry, autoConstants.startPoseBlueRight);
        autoConstants = new AutoConstants();



        elevator = new Elevator(true);
        intake = new Intake();
        intakeExtension = new IntakeExtension(gamepad1, true);
        claw = new Claw();
        outtake = new Outtake();

        this.depositActions = new DepositActions(elevator, intake, claw, outtake, intakeExtension);
        this.intakeActions = new IntakeActions(intake, intakeExtension, claw);
        this.updateActions = new UpdateActions(elevator, intake, claw, outtake, intakeExtension);

        subsystemActions = new SubsystemActions(this.depositActions, this.intakeActions, this.updateActions);

        initCamera();
        webcam.setPipeline(propPipelineBlueRight);

        while (opModeInInit() && !isStopRequested()) {
            betterGamepad2.update();
            intake.setAngle(Intake.Angle.MID);
            intakeExtension.setTarget(0);
            intake.updateClawState(Intake.ClawState.CLOSE, ClawSide.BOTH);
            claw.updateState(Claw.ClawState.OPEN, ClawSide.BOTH);
            outtake.setAngle(Outtake.Angle.INTAKE);
            telemetry.addData("POS", propLocation.name());
            telemetry.addData("elevator pos", tempHeight);
            telemetry.update();

            switch (propPipelineBlueRight.getLocation()) {
                case Left:
                    propLocation = AutoSettings.PropLocation.LEFT;
                    break;
                case Right:
                    propLocation = AutoSettings.PropLocation.RIGHT;
                    break;
                case Center:
                    propLocation = AutoSettings.PropLocation.MIDDLE;
                    break;
            }

            if (betterGamepad2.dpadRightOnce()) {
                tempHeight = tempHeightMax;
            } else if (betterGamepad2.dpadLeftOnce()) {
                tempHeight = tempHeightMin;
            }

            if (betterGamepad2.dpadUpOnce()) {
                if (first) {
                    webcam.stopStreaming();
                    first = false;
                }

                propLocation = cycleVision(propLocation);
            } else if (betterGamepad2.dpadDownOnce()) {
                initCamera();
                webcam.setPipeline(propPipelineBlueRight);
            }
        }

        waitForStart();
        webcam.stopStreaming();

        if (isStopRequested()) return;

        time.reset();

        switch (propLocation) {

            case RIGHT:
                runBlocking(new ParallelAction(
                        new AutoBlueRightRight(subsystemActions).generateTraj(),
                        updateActions.updateSystems()
                ));
                break;

        }
    }


}
