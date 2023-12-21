package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Elevator {

    private final RobotHardware robot;
    public static double ELEVATOR_INCREMENT = 70;
    public static double BASE_LEVEL = 800;
    public static double MAX_LEVEL = 1600;
    double currentTarget = 0;
    boolean usePID = true;
    public static double maxPower = 1;
    public static double kP = 0.005, kI = 0, kD = 0.0000001;

    Gamepad gamepad;
    BetterGamepad cGamepad;
    PIDFController controller, controller2;
    PIDFController.PIDCoefficients pidCoefficients = new PIDFController.PIDCoefficients();

    boolean isAuto;
    public Elevator(Gamepad gamepad)
    {
        this.robot = RobotHardware.getInstance();

        this.gamepad = gamepad;
        this.cGamepad = new BetterGamepad(gamepad);

        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);
        controller2 = new PIDFController(pidCoefficients);

        isAuto = false;
    }

    public Elevator()
    {
        this.robot = RobotHardware.getInstance();

        pidCoefficients.kP = kP;
        pidCoefficients.kI = kI;
        pidCoefficients.kD = kD;

        controller = new PIDFController(pidCoefficients);
        controller2 = new PIDFController(pidCoefficients);

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
                // TODO: CHANGE TO LEFT!!!!!
                if(gamepad.left_stick_y != 0 && !gamepad.left_stick_button)
                {
                    robot.elevatorMotorRight.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
                    robot.elevatorMotorLeft.setPower(Range.clip(-gamepad.left_stick_y, -maxPower, maxPower));
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

    public void updateTest() {

        if(!isAuto)
        {
            cGamepad.update();

            if (usePID)
            {
                setPidControl();
            }
            else
            {
                if(gamepad.right_stick_y != 0 && !gamepad.left_stick_button)
                {
                    robot.elevatorMotorRight.setPower(Range.clip(-gamepad.right_stick_y, -maxPower, maxPower));
                    robot.elevatorMotorLeft.setPower(Range.clip(-gamepad.right_stick_y, -maxPower, maxPower));
                }
                else if(gamepad.right_stick_y != 0 && gamepad.left_stick_button)
                {
                    robot.elevatorMotorRight.setPower(Range.clip(-gamepad.right_stick_y, -maxPower/2, maxPower/2));
                    robot.elevatorMotorLeft.setPower(Range.clip(-gamepad.right_stick_y, -maxPower/2, maxPower/2));
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

        robot.elevatorMotorRight.setTargetPosition((int)currentTarget);
        robot.elevatorMotorLeft.setTargetPosition((int)currentTarget);

        robot.elevatorMotorRight.setPower(1);
        robot.elevatorMotorLeft.setPower(1);

        robot.elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void setPidControl()
    {
        controller.updateError(currentTarget - robot.elevatorMotorRight.getCurrentPosition());
        controller2.updateError(currentTarget - robot.elevatorMotorLeft.getCurrentPosition());

        robot.elevatorMotorRight.setPower(controller.update());
        robot.elevatorMotorLeft.setPower(controller2.update());
    }

    public void setTarget(double target)
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

    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    public void setAuto(boolean isAuto)
    {
        this.isAuto = isAuto;
    }
}