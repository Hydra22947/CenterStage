package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static double maxPower = 0.75;
    public static double kPR = 0.0075, kIR = 0, kDR = 0.01;
    public static double kPL = 0.005, kIL = 0, kDL = 0.01;
    public DcMotorEx elevatorMotorRight;
    public DcMotorEx elevatorMotorLeft;
    Gamepad gamepad;
    BetterGamepad cGamepad;
    PIDFController controllerR, controllerL;
    PIDFController.PIDCoefficients pidCoefficientsR = new PIDFController.PIDCoefficients();
    PIDFController.PIDCoefficients pidCoefficientsL = new PIDFController.PIDCoefficients();

    boolean isAuto, firstPID = false;
    public Elevator(Gamepad gamepad, boolean isAuto, boolean firstPID)
    {
        this.isAuto = isAuto;
        this.elevatorMotorRight = robot.hardwareMap.get(DcMotorEx.class, "mER");
        this.elevatorMotorLeft = robot.hardwareMap.get(DcMotorEx.class, "mEL");
        elevatorMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        if(firstPID && !isAuto)
        {
            elevatorMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if(firstPID && isAuto)
        {
            elevatorMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if (isAuto)
        {
            elevatorMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else
        {
            elevatorMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        this.elevatorMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.elevatorMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    }

    public Elevator(boolean isAuto)
    {
        this.isAuto = isAuto;

        this.elevatorMotorRight = robot.hardwareMap.get(DcMotorEx.class, "mER");
        this.elevatorMotorLeft = robot.hardwareMap.get(DcMotorEx.class, "mEL");
        elevatorMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        if (isAuto)
        {
            elevatorMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else
        {
            elevatorMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        this.elevatorMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.elevatorMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidCoefficientsR.kP = kPR;
        pidCoefficientsR.kI = kIR;
        pidCoefficientsR.kD = kDR;

        pidCoefficientsL.kP = kPL;
        pidCoefficientsL.kI = kIL;
        pidCoefficientsL.kD = kDL;

        controllerR = new PIDFController(pidCoefficientsR);
        controllerL = new PIDFController(pidCoefficientsL);
    }

    public void setFirstPID(boolean firstPID) {
        this.firstPID = firstPID;
    }

    public void update() {

        if(!firstPID)
        {
            cGamepad.update();

            if (usePID)
            {
                setPidControl();
            }
            else
            {
                elevatorMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if(gamepad.right_stick_y != 0 && !gamepad.right_stick_button)
                {
                    if((-gamepad.right_stick_y) < 0)
                    {
                        elevatorMotorRight.setPower(Range.clip(-gamepad.right_stick_y, -maxPower/2, maxPower/2));
                        elevatorMotorLeft.setPower(Range.clip(-gamepad.right_stick_y, -maxPower/2, maxPower/2));
                    }
                    else
                    {
                        elevatorMotorRight.setPower(Range.clip(-gamepad.right_stick_y, -maxPower, maxPower));
                        elevatorMotorLeft.setPower(Range.clip(-gamepad.right_stick_y, -maxPower, maxPower));
                    }
                }
                else if(gamepad.right_stick_y != 0 && gamepad.right_stick_button)
                {
                    elevatorMotorRight.setPower(Range.clip(-gamepad.right_stick_y, -maxPower/2, maxPower/2));
                    elevatorMotorLeft.setPower(Range.clip(-gamepad.right_stick_y, -maxPower/2, maxPower/2));
                }
                else
                {
                    elevatorMotorRight.setPower(0);
                    elevatorMotorLeft.setPower(0);
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
        elevatorMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevatorMotorRight.setTargetPosition((int)currentTargetRight);
        elevatorMotorLeft.setTargetPosition((int)currentTargetLeft);

        elevatorMotorRight.setPower(1);
        elevatorMotorLeft.setPower(1);

        elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setPidControl()
    {
        controllerR.updateError(currentTargetRight - elevatorMotorRight.getCurrentPosition());
        controllerL.updateError(currentTargetLeft - elevatorMotorLeft.getCurrentPosition());

        elevatorMotorRight.setPower(controllerR.update());
        elevatorMotorLeft.setPower(controllerL.update());
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
        return elevatorMotorRight.getCurrentPosition();
    }

    public double getPosLeft()
    {
        return elevatorMotorLeft.getCurrentPosition();
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