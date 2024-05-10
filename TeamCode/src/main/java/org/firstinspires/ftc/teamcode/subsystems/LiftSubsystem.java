package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class LiftSubsystem extends SubsystemBase
{

    private final RobotHardware robot = RobotHardware.getInstance();
    public static double ELEVATOR_INCREMENT = 70;
    public static double MAX_LEVEL = 3450;
    public static double HANG_OPEN = 3150;
    public static double HANG = 500;
    double currentTargetRight = 0, currentTargetLeft = 0;
    boolean usePID = true;
    public static double maxPower = 0.75;
    public static double kPR = 0.0075, kIR = 0, kDR = 0.01;
    public static double kPL = 0.005, kIL = 0, kDL = 0.01;
    Gamepad gamepad;
    PIDFController controllerR, controllerL;
    PIDFController.PIDCoefficients pidCoefficientsR = new PIDFController.PIDCoefficients();
    PIDFController.PIDCoefficients pidCoefficientsL = new PIDFController.PIDCoefficients();

    public LiftSubsystem(Gamepad gamepad, boolean resetEncoders)
    {
        if (resetEncoders)
        {
            robot.liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.gamepad = gamepad;

        pidCoefficientsR.kP = kPR;
        pidCoefficientsR.kI = kIR;
        pidCoefficientsR.kD = kDR;

        pidCoefficientsL.kP = kPL;
        pidCoefficientsL.kI = kIL;
        pidCoefficientsL.kD = kDL;

        controllerR = new PIDFController(pidCoefficientsR);
        controllerL = new PIDFController(pidCoefficientsL);

    }

    public LiftSubsystem(boolean resetEncoders)
    {
        if (resetEncoders)
        {
            robot.liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        robot.liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidCoefficientsR.kP = kPR;
        pidCoefficientsR.kI = kIR;
        pidCoefficientsR.kD = kDR;

        pidCoefficientsL.kP = kPL;
        pidCoefficientsL.kI = kIL;
        pidCoefficientsL.kD = kDL;

        controllerR = new PIDFController(pidCoefficientsR);
        controllerL = new PIDFController(pidCoefficientsL);
    }

    public void update() {
        if (usePID)
        {
            setPidControl();
        }
        else
        {
            robot.liftMotorRight.setPower(Range.clip(-gamepad.right_stick_y, -maxPower, maxPower));
            robot.liftMotorLeft.setPower(Range.clip(-gamepad.right_stick_y, -maxPower, maxPower));
        }
    }

    public void setPidControl()
    {
        controllerR.updateError(currentTargetRight - robot.liftMotorRight.getCurrentPosition());
        controllerL.updateError(currentTargetLeft - robot.liftMotorLeft.getCurrentPosition());

        robot.liftMotorRight.setPower(controllerR.update());
        robot.liftMotorLeft.setPower(controllerL.update());
    }

    public void setTarget(double target)
    {
        if(target > MAX_LEVEL)
        {
            this.currentTargetRight = MAX_LEVEL;
            this.currentTargetLeft = MAX_LEVEL;
        }
        else
        {
            this.currentTargetRight = target;
            this.currentTargetLeft = target;
        }
    }
    //Move elevator for auto
    public void move(double target)
    {
        this.setTarget(target);
        this.setPidControl();
    }
    public void setTarget(double targetRight, double targetLeft)
    {
        if(targetRight > MAX_LEVEL)
        {
            this.currentTargetRight = MAX_LEVEL;
        }
        else
        {
            this.currentTargetRight = targetRight;
        }

        if(targetLeft > MAX_LEVEL)
        {
            this.currentTargetLeft = MAX_LEVEL;
        }
        else
        {
            this.currentTargetLeft = targetLeft;
        }
    }

    public double getTargetRight()
    {
        return this.currentTargetRight;
    }

    public double getTargetLeft()
    {
        return this.currentTargetLeft;
    }

    public double getPosRight()
    {
        return robot.liftMotorRight.getCurrentPosition();
    }

    public double getPosLeft()
    {
        return robot.liftMotorLeft.getCurrentPosition();
    }

    public double getPos()
    {
        return (getPosLeft() + getPosRight()) / 2;
    }

    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    public PIDFController getControllerR() {
        return controllerR;
    }

    public PIDFController getControllerL() {
        return controllerL;
    }
}