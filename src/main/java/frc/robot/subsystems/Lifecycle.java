package frc.robot.subsystems;

public interface Lifecycle {
     default void teleopInit() {}

    default void autoInit() {}
}
