package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class DroneSubsystem extends SubsystemBase {

    RobotHardware robot = RobotHardware.getInstance();

    public enum DroneState {
        SHOT,
        LOADED
    }

    public DroneState state = DroneState.LOADED;

    // LOOK FORM INTAKE
    public static double shot = 0.5, loaded = .2;

    public DroneSubsystem() {

    }

    public void toggle()
    {
        switch (state)
        {
            case SHOT:
                robot.planeServo.setPosition(loaded);
                break;
            case LOADED:
                robot.planeServo.setPosition(shot);
                break;
        }
    }


}