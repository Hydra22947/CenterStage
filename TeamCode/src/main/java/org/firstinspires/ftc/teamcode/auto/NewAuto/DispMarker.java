package org.firstinspires.ftc.teamcode.auto.NewAuto;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.testing.hardware.AxonCRTest;

import java.util.Vector;

public class DispMarker implements Action {

    private Action _systemFunction;
    private Vector2d _targetPose;
    private MecanumDrive _drive;
    public static final double EPSILON = 2; // IN INCHES

    public DispMarker(Vector2d targetPose, MecanumDrive drivetrain, Action systemFunction) {
        this._targetPose = targetPose;
        this._drive = drivetrain;
        this._systemFunction = systemFunction;
    }

    private boolean isClose() {
        return Math.abs(this._drive.pose.position.x - this._targetPose.x) <= EPSILON ||
                Math.abs(this._drive.pose.position.y - this._targetPose.y) <= EPSILON;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        if (isClose()) {
            telemetryPacket.addLine("The robot is close to position");
            runBlocking(this._systemFunction);
            return false;
        }
        return true;
    }
}
