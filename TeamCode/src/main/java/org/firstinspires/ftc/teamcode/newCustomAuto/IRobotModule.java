package org.firstinspires.ftc.teamcode.newCustomAuto;

public interface IRobotModule {
    public static boolean ENABLED = false;
    default void initUpdate() {}
    default void atStart() {}
    void update();
    default void emergencyStop() {}
}