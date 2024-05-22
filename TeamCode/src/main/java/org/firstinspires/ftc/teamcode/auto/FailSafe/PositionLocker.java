package org.firstinspires.ftc.teamcode.auto.FailSafe;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Angle;

public class PositionLocker {

    private static MecanumDrive _drivetrain;

    public PositionLocker(MecanumDrive drivetrain) {
        this._drivetrain = drivetrain;
    }

    public class LockPositionAction implements Action {

        private Pose2d _targetPose;

        public LockPositionAction(Pose2d targetPose) {
            this._targetPose = targetPose;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Pose2d currentPose = _drivetrain.pose;
            Pose2d difference = Pose2d.exp(this._targetPose.minus(currentPose));
            Vector2d rotatedVector = Rotation2d.exp(-currentPose.heading.log()).inverse().times(difference.position);

            double heading = Angle.normDelta(this._targetPose.heading.toDouble())- Angle.normDelta(currentPose.heading.toDouble());
            _drivetrain.setDrivePowers(new PoseVelocity2d(rotatedVector, heading));

            _drivetrain.updatePoseEstimate();
            return false;
        }
    }


}
