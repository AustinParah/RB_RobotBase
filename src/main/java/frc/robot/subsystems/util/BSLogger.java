package frc.robot.subsystems.util;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

public class BSLogger {
    public static void log(String origin, String message) {
        DataLogManager.log("%.3f [%s] %s".formatted(Timer.getFPGATimestamp(), origin, message));
    }
}
