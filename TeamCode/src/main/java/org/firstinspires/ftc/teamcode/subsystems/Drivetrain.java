package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;
import org.firstinspires.ftc.teamcode.Globals;

@Config
public class Drivetrain {

    boolean redAlliance;

    private BetterGamepad _cGamepad1;

    private final RobotHardware robot;
    //BasicPID pid;
    //AngleController controller;
    //PIDCoefficients coefficients;

    enum Wheels
    {
        frontLeft,
        frontRight,
        backLeft,
        backRight
    }

    double frontLeftPower = 0, backLeftPower = 0, frontRightPower = 0, backRightPower = 0;
    //public static double kP = 1, kI = 0, kD = 0;
    public static double targetAngle = 0;
    double rotationLock = 0;

    //Constructor
    public Drivetrain(Gamepad gamepad1, boolean redAlliance)
    {
        this.robot = RobotHardware.getInstance();

//        coefficients.Kp = kP;
//        coefficients.Ki = kI;
//        coefficients.Kd = kD;
//
//        pid = new BasicPID(coefficients);
//        controller = new AngleController(pid);

        // gamepad helper to see if pressed button once
        this._cGamepad1 = new BetterGamepad(gamepad1);

        this.redAlliance = redAlliance;

        resetAngle();
    }

    public void update() {
        _cGamepad1.update();
        double heading = robot.getAngle();

        double twist = Range.clip(_cGamepad1.right_stick_x * 1.1, -Globals.MAX_POWER, Globals.MAX_POWER);
        double strafeSpeed = Range.clip(_cGamepad1.left_stick_x, -Globals.MAX_POWER, Globals.MAX_POWER);
        double forwardSpeed = Range.clip(-_cGamepad1.left_stick_y, -Globals.MAX_POWER, Globals.MAX_POWER);

        if(_cGamepad1.XOnce())
        {
            resetAngle();
        }

//        if (_cGamepad1.YOnce())
//        {
//            rotationLock = controller.calculate(targetAngle, robot.getAngle());
//        }
//        else
//        {
//            rotationLock = 0;
//        }

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-heading);

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[Wheels.frontLeft.ordinal()] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[Wheels.frontRight.ordinal()] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[Wheels.backLeft.ordinal()] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[Wheels.backRight.ordinal()] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[Wheels.frontLeft.ordinal()] += (twist + rotationLock);
        wheelSpeeds[Wheels.frontRight.ordinal()] -= (twist + rotationLock);
        wheelSpeeds[Wheels.backLeft.ordinal()] += (twist + rotationLock);
        wheelSpeeds[Wheels.backRight.ordinal()] -= (twist + rotationLock);

        normalize(wheelSpeeds);

        robot.dtFrontLeftMotor.setPower(wheelSpeeds[Wheels.frontLeft.ordinal()]);
        robot.dtFrontRightMotor.setPower(wheelSpeeds[Wheels.frontRight.ordinal()]);
        robot.dtBackLeftMotor.setPower(wheelSpeeds[Wheels.backLeft.ordinal()]);
        robot.dtBackRightMotor.setPower(wheelSpeeds[Wheels.backRight.ordinal()]);

        robot.telemetry.addData("Heading", Math.toDegrees(heading));
        robot.telemetry.addData("fl", frontLeftPower);
        robot.telemetry.addData("bl", backLeftPower);
        robot.telemetry.addData("fr", frontRightPower);
        robot.telemetry.addData("br", backRightPower);
    }

    void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }

    }

    /**
     * Normalize the wheel speeds
     */
    void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }

    }

    public void resetAngle()
    {
        // check if we are blue/red alliance and set zero angle - For centric drive
        if(!redAlliance)
        {
            robot.setImuOffset(-(Math.PI + Math.PI/2));
        }
        else if(redAlliance)
        {
            robot.setImuOffset(Math.PI + Math.PI/2);
        }
        robot.telemetry.addData("RESET", 0);
        robot.telemetry.update();
    }

}
