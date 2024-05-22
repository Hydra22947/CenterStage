//package org.firstinspires.ftc.teamcode.auto.FailSafe;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Trajectory;
//import com.acmerobotics.roadrunner.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.auto.Actions.IntakeActions;
//
//public class IntakeFailSafe implements Action {
//
//    private RobotHardware robot = RobotHardware.getInstance();
//    private IntakeActions _intakeActions;
//    private Vector2d _intakePose;
//
//    public IntakeFailSafe(IntakeActions intakeActions, Vector2d intakePose) {
//        this._intakeActions = intakeActions;
//        this._intakePose = intakePose;
//    }
//
//    public void fixPosition() {
//        robot.drive.actionBuilder(robot.drive.pose)
//                .stopAndAdd(this._intakeActions.closeExtension())
//                .strafeToLinearHeading(this._intakePose, Math.toRadians(0))
//                .build();
//    }
//
//    @Override
//    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//        if (!(this._intakeActions.getIntake().checkIfPixelInRight(robot.colorRight) ||
//                this._intakeActions.getIntake().checkIfPixelIn(robot.colorLeft))) {
//            fixPosition();
//            return true;
//        }
//
//        return false;
//    }
//}
