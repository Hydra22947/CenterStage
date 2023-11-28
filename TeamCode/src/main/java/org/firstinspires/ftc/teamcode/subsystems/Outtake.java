package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;
import org.jetbrains.annotations.NotNull;

@Config
public class Outtake extends BetterSubsystem {

    private final RobotHardware robot;

    public static double intakeHandPivot = 0.375, intakeClawPivot = 0.135;
    public static double outtakeHandPivot = 0.9, outtakeClawPivot = 0.97;

    public static double power = 1;

    public enum Angle
    {
        INTAKE,
        OUTTAKE
    }

    public enum Type
    {
        CLAW,
        HAND
    }

    Angle angle = Angle.INTAKE;
    Intake intake = null;
    Claw claw = null;

    public Outtake(Intake intake, Claw claw)
    {
        this.robot = RobotHardware.getInstance();
        this.intake = intake;
        this.claw = claw;
    }

    public Outtake()
    {
        this.robot = RobotHardware.getInstance();

    }

    @Override
    public void periodic() {
        updateState(Type.CLAW);
        updateState(Type.HAND);

        if(intake != null && claw != null && robot.isReadyToTransferPixels() && angle == Angle.INTAKE)
        {
            robot.telemetry.update();
            intake.intakeMove(Intake.powerTransfer);
            // until pixels see beam(already happens in opmode) because of peridoric
            if(!claw.checkAndClose(robot.breakbeamRight, ClawSide.RIGHT) && !claw.checkAndClose(robot.breakbeamLeft, ClawSide.LEFT));

        }
    }

    @Override
    public void read() {

    }

    @Override
    public void write() {

    }

    @Override
    public void reset() {

    }


    public void setAngle(@NotNull Angle angle) {
        this.angle = angle;
    }

    public void updateState(@NotNull Type type) {

        switch(type) {
            case CLAW:
                switch (angle){
                    case INTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(intakeClawPivot);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeClawPivotServo.setPosition(outtakeClawPivot);
                        break;
                }
                break;
            case HAND:
                switch (angle){
                    case INTAKE:
                        this.robot.outtakeHandRightServo.setPosition(intakeHandPivot);
                        this.robot.outtakeHandLeftServo.setPosition(intakeHandPivot);
                        break;
                    case OUTTAKE:
                        this.robot.outtakeHandRightServo.setPosition(outtakeHandPivot);
                        this.robot.outtakeHandLeftServo.setPosition(outtakeHandPivot);
                        break;
                }
                break;
        }
    }

}