package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.Globals;

@Config
public class Drivetrain {

    boolean redAlliance;

    private BetterGamepad _cGamepad1;

    private final RobotHardware robot;
    Vector2d input;
    PoseVelocity2d powers;
    public static double maxPower = 0.7;
    public static double minPower = 0.3;
    public static double maxPowerSpin = maxPower / 1.5;
    double power = 0;
    boolean slow = false;

    //Constructor
    public Drivetrain(Gamepad gamepad1, boolean redAlliance)
    {
        this.robot = RobotHardware.getInstance();

        // gamepad helper to see if pressed button once
        this._cGamepad1 = new BetterGamepad(gamepad1);

        this.redAlliance = redAlliance;

        resetAngle();
    }

    public void update() {
        _cGamepad1.update();

        input = new Vector2d(
                _cGamepad1.left_stick_y * maxPower,
                -_cGamepad1.left_stick_x * maxPower
        );

        input = Rotation2d.exp(-robot.getAngle()).inverse().times(
                new Vector2d(input.x, input.y));

        powers = new PoseVelocity2d(
                input,
                _cGamepad1.right_stick_x * maxPower
        );



        if(_cGamepad1.XOnce())
        {
            resetAngle();
        }

        if(_cGamepad1.right_trigger != 0 && slow)
        {
            power = minPower;
            maxPowerSpin = minPower / 2;
        }
        else
        {
            power = maxPower;
            maxPowerSpin = maxPower / 2;
        }

        robot.dtFrontLeftMotor.setPower(powers.linearVel.x + powers.angVel + powers.linearVel.y);
        robot.dtBackLeftMotor.setPower(powers.linearVel.x  + powers.angVel - powers.linearVel.y);
        robot.dtFrontRightMotor.setPower(powers.linearVel.x  - powers.angVel - powers.linearVel.y);
        robot.dtBackRightMotor.setPower(powers.linearVel.x  - powers.angVel + powers.linearVel.y);

        robot.telemetry.addData("heading H", Math.toDegrees(robot.getAngle()));
        robot.telemetry.addData("offset", robot.getImuOffset());
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

    public void setSlow(boolean slow) {
        this.slow = slow;
    }
}
