package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.ClawSide;
import org.jetbrains.annotations.NotNull;

@Config
public class Plane  {

    RobotHardware robot = RobotHardware.getInstance();

    public enum PlaneState {
        SHOT,
        LOADED
    }

    public PlaneState state = PlaneState.LOADED;

    // LOOK FORM INTAKE
    public static double shot = 0.5, loaded = .2;

    public Plane() {

    }

    public void update() {
        switch (state)
        {
            case SHOT:
                robot.planeServo.setPosition(shot);
                break;
            case LOADED:
                robot.planeServo.setPosition(loaded);
                break;
        }
    }

    public void toggle()
    {
        switch (state)
        {
            case SHOT:
                state = PlaneState.LOADED;
                break;
            case LOADED:
                state = PlaneState.SHOT;
                break;
        }
    }


}