package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;


import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;
@Config

public class StandardTwoTrackingWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.944882; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    // left
    public static double LEFT_X = 0; // X is the up and down direction
    public static double LEFT_Y = 4.823; // Y is the strafe direction

    // front
    public static double FRONT_X = 7.087;
    public static double FRONT_Y = 0;

    public static double X_MULTIPLIER = 0.9887182205; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.002845684842; // Multiplier in the Y direction

    private Encoder leftEncoder, frontEncoder;


    private SampleMecanumDrive drive;

    public static boolean reverseLeft = false, reverseFront = true;
    public StandardTwoTrackingWheelLocalizer(SampleMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(LEFT_X, LEFT_Y, 0),
                new Pose2d(FRONT_X, FRONT_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        leftEncoder = RobotHardware.getInstance().podLeft;
        frontEncoder = RobotHardware.getInstance().podFront;

        if(reverseLeft) leftEncoder.setDirection(Encoder.Direction.REVERSE);
        if(reverseFront) frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getRawVelocity()) * Y_MULTIPLIER
        );
    }
}