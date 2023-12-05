package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
public class LocalizationTest extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    public static double power = 0.5;
    public static double powerSpin = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), robot);

            waitForStart();

            while (opModeIsActive()) {

                Vector2d input = new Vector2d(
                        -gamepad1.left_stick_y * power,
                        -gamepad1.left_stick_x * power
                );

                input = drive.pose.heading.inverse().times(
                        new Vector2d(-input.x, input.y));

                drive.setDrivePowers(new PoseVelocity2d(
                        input,
                        -gamepad1.right_stick_x * powerSpin
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();
            }
        } else {
            throw new AssertionError();
        }
    }
}
