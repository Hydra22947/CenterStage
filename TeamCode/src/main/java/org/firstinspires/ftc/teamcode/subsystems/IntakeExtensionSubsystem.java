package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class IntakeExtensionSubsystem extends SubsystemBase
{

    private final RobotHardware robot = RobotHardware.getInstance();
    public static double MAX_LEVEL = 1250;
    double currentTarget = 0;
    boolean usePID = true;
    int maxPower = 1;
    public static double kP = 0.008, kI = 0, kD = 0.01;

    Gamepad gamepad;
    PIDFController controller;
    PIDFController.PIDCoefficients pidCoefficients = new PIDFController.PIDCoefficients();

    public IntakeExtensionSubsystem(Gamepad gamepad, boolean resetEncoders)
    {
        // EXTENSION
        if(resetEncoders)
        {
            robot.intakeExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.intakeExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else
        {
            robot.intakeExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

        this.gamepad = gamepad;

        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);
    }

    public IntakeExtensionSubsystem()
    {
        robot.intakeExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeExtensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);
    }

    @Override
    public void periodic() {
        if (usePID)
        {
            setPidControl();
        }
        else
        {
            robot.intakeExtensionMotor.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
        }
    }

    public void setPidControl()
    {
        controller.updateError(currentTarget - robot.intakeExtensionMotor.getCurrentPosition());

        robot.intakeExtensionMotor.setPower(controller.update());
    }


    public void setTarget(int target)
    {
        if(target > MAX_LEVEL)
        {
            this.currentTarget = MAX_LEVEL;
        }
        else
        {
            this.currentTarget = target;
        }
    }


    public double getTarget()
    {
        return this.currentTarget;
    }

    public double getPos()
    {
        return robot.intakeExtensionMotor.getCurrentPosition();
    }


    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    public PIDFController getController() {
        return controller;
    }
}