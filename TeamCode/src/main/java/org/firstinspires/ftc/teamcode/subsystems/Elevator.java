package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterSubsystem;

@Config
public class Elevator {

    private final RobotHardware robot;
    public static double BASE_LEVEL = 750;
    public static double MAX_LEVEL = 1650;
    double currentTarget = 0;
    boolean usePID = true;
    public static double maxPower = 0.85;
    public static double kP = 0.005, kI = 0, kD = 0.0000001;

    Gamepad gamepad;
    BetterGamepad cGamepad;
    PIDFController controller, controller2;
    PIDFController.PIDCoefficients pidCoefficients = new PIDFController.PIDCoefficients();

    boolean first = true, firstNoPID = true;
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
                if(first)
                {
                    robot.elevatorMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    first = false;
                    firstNoPID = true;
                }


                setPidControl();
            }
            else
            {
                if(firstNoPID)
                {
                    robot.elevatorMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    firstNoPID = false;
                    first = true;
                }

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
            setPidControl();
        }
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
        setPidControl();
        if(target > MAX_LEVEL)
        {
            this.currentTarget = MAX_LEVEL;
        }
        else
        {
            this.currentTarget = target;
        }

    }

    public void setUsePID(boolean usePID) {
        this.usePID = usePID;
    }

    public void setAuto(boolean isAuto)
    {
        this.isAuto = isAuto;
    }
}