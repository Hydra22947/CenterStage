package org.firstinspires.ftc.teamcode.newCustomAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "Super Custom Auto")
@Config
public class SuperCustomAuto extends LinearOpMode {
    RobotHardware robot = RobotHardware.getInstance();

    MecanumDrive drive;
    FtcDashboard dash;

    public static double x = 0, y = 0, heading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        robot.init(hardwareMap, telemetry);
        dash = FtcDashboard.getInstance();


        drive = new MecanumDrive(new Localizer(robot, new Pose()), MecanumDrive.RunMode.PID, false);


        while(opModeInInit() && !isStopRequested()){
            telemetry.update();
        }

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        loopTimer.startTime();

        while (opModeIsActive() && !isStopRequested()) {
            drive.setRunMode(MecanumDrive.RunMode.PID);
            drive.update();

            drive.setTargetPose(new Pose(x,y,heading));

            telemetry.addData("Imu angle", drive.getLocalizer().getHeading());
            telemetry.addData("Hz", 1.0/loopTimer.seconds());

            loopTimer.reset();

            telemetry.update();
        }
    }
}