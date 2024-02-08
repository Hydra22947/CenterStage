package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Elevator implements Subsystem
{

    private final RobotHardware robot = RobotHardware.getInstance();
    public static double ELEVATOR_INCREMENT = 70;
    public static double BASE_LEVEL = 1000;
    public static double MAX_LEVEL = 3450;
    public static double HANG_OPEN = 3150;
    public static double HANG = 500;
    double currentTargetRight = 0, currentTargetLeft = 0;
    boolean usePID = true;
    public static double maxPower = .5;
    public static double kPR = 0.0075, kIR = 0, kDR = 0.01;
    public static double kPL = 0.005, kIL = 0, kDL = 0.01;

    Gamepad gamepad;
    BetterGamepad cGamepad;
    PIDFController controllerR, controllerL;
    PIDFController.PIDCoefficients pidCoefficientsR = new PIDFController.PIDCoefficients();
    PIDFController.PIDCoefficients pidCoefficientsL = new PIDFController.PIDCoefficients();

    boolean isAuto;
    public Elevator(Gamepad gamepad)
    {
        this.gamepad = gamepad;
        this.cGamepad = new BetterGamepad(gamepad);

        pidCoefficientsR.kP = kPR;
        pidCoefficientsR.kI = kIR;
        pidCoefficientsR.kD = kDR;

        pidCoefficientsL.kP = kPL;
        pidCoefficientsL.kI = kIL;
        pidCoefficientsL.kD = kDL;

        controllerR = new PIDFController(pidCoefficientsR);
        controllerL = new PIDFController(pidCoefficientsL);

        isAuto = false;
    }

    public Elevator()
    {
        pidCoefficientsR.kP = kPR;
        pidCoefficientsR.kI = kIR;
        pidCoefficientsR.kD = kDR;

        pidCoefficientsL.kP = kPL;
        pidCoefficientsL.kI = kIL;
        pidCoefficientsL.kD = kDL;

        controllerR = new PIDFController(pidCoefficientsR);
        controllerL = new PIDFController(pidCoefficientsL);

        isAuto = true;
    }

    public void update() {

        if(!isAuto)
        {
            cGamepad.update();

            if (usePID)
            {
                setPidControl();
            }
            else
            {
                if(gamepad.left_stick_y != 0 && !gamepad.left_stick_button)
                {
                    if((-gamepad.left_stick_y) < 0)
                    {
                        robot.elevatorMotorRight.setPower(Range.clip(-gamepad.left_stick_y, -maxPower/2, maxPower/2));
                        robot.elevatorMotorLeft.setPower(Range.clip(-gamepad.left_stick_y, -maxPower/2, maxPower/2));
                    }
                    else
                    {
                        robot.elevatorMotorRight.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
                        robot.elevatorMotorLeft.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
                    }
                }
                else if(gamepad.left_stick_y != 0 && gamepad.left_stick_button)
                {
                    robot.elevatorMotorRight.setPower(Range.clip(-gamepad.left_stick_y, -maxPower/2, maxPower/2));
                    robot.elevatorMotorLeft.setPower(Range.clip(-gamepad.left_stick_y, -maxPower/2, maxPower/2));
                }
                else
                {
                    robot.elevatorMotorRight.setPower(0);
                    robot.elevatorMotorLeft.setPower(0);
                }

            }
        }
        else
        {
            firstPID();
        }
    }

    void firstPID()
    {
        robot.elevatorMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.elevatorMotorRight.setTargetPosition((int)currentTargetRight);
        robot.elevatorMotorLeft.setTargetPosition((int)currentTargetLeft);

        robot.elevatorMotorRight.setPower(1);
        robot.elevatorMotorLeft.setPower(1);

        robot.elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void setPidControl()
    {
        controllerR.updateError(currentTargetRight - robot.elevatorMotorRight.getCurrentPosition());
        controllerL.updateError(currentTargetLeft - robot.elevatorMotorLeft.getCurrentPosition());

        robot.elevatorMotorRight.setPower(controllerR.update());
        robot.elevatorMotorLeft.setPower(controllerL.update());
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
        this.update();
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
        return robot.elevatorMotorRight.getCurrentPosition();
    }

    public double getPosLeft()
    {
        return robot.elevatorMotorLeft.getCurrentPosition();
    }

    public double getPos()
    {
        return (getPosLeft() + getPosRight()) / 2;
    }

    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    public void setAuto(boolean isAuto)
    {
        this.isAuto = isAuto;
    }

    @Override
    public void play() {

    }

    @Override
    public void loop(boolean allowMotors) {
        isAuto = true;
        update();
    }

    @Override
    public void stop() {

    }

    public PIDFController getControllerR() {
        return controllerR;
    }

    public PIDFController getControllerL() {
        return controllerL;
    }
}