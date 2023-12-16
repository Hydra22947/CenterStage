package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@Config
public class Drivetrain {

    boolean redAlliance;

    private BetterGamepad _cGamepad1;

    private final RobotHardware robot;
    Vector2d input;
    public static double maxPower = 0.6;
    double power = 0, twist = 0;
    boolean slow = false;

    //Constructor
    public Drivetrain(Gamepad gamepad1, boolean redAlliance)
    {
        this.robot = RobotHardware.getInstance();

        // gamepad helper to see if pressed button once
        this._cGamepad1 = new BetterGamepad(gamepad1);

        power = maxPower;

        this.redAlliance = redAlliance;

        resetAngle();
    }

    public void update() {
        _cGamepad1.update();

        input = new Vector2d(
                Range.clip(_cGamepad1.left_stick_y, -power, power),
                Range.clip(-_cGamepad1.left_stick_x, -power, power)
        ).rotated(robot.getAngle());

        twist = Range.clip(_cGamepad1.right_stick_x, -power * 0.8, power* 0.8);

        if(_cGamepad1.XOnce())
        {
            resetAngle();
        }

        robot.dtFrontLeftMotor.setPower(input.getX() + twist + input.getY());
        robot.dtBackLeftMotor.setPower(input.getX()  + twist - input.getY());
        robot.dtFrontRightMotor.setPower(input.getX()  - twist - input.getY());
        robot.dtBackRightMotor.setPower(input.getX()  - twist + input.getY());

        robot.telemetry.addLine("" + slow);
        robot.telemetry.addData("heading H", Math.toDegrees(robot.getAngle()));
        robot.telemetry.addData("offset", robot.getImuOffset());
        robot.telemetry.addData("power", power);
        robot.telemetry.addData("max power", maxPower);
    }


    public void resetAngle()
    {
        robot.imu.resetYaw();

        // check if we are blue/red alliance and set zero angle - For centric drive
        if(!redAlliance)
        {
            robot.setImuOffset(-Math.PI);
        }
        else if(redAlliance)
        {
            robot.setImuOffset(Math.PI);
        }

        robot.telemetry.update();
    }

    public void fast()
    {
        power = maxPower;
    }

    public void slow()
    {
        power = maxPower / 2;
    }
}
