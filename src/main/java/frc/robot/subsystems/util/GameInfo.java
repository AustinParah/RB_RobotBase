package frc.robot.subsystems.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;

public class GameInfo {
    public static boolean isRedAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public static boolean isBlueAlliance() {
        return !isRedAlliance();
    }
}