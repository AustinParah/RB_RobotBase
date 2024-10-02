package frc.robot.config;

/**
 * Represents specific configuration options for the robot, such as motor CAN IDs.
 */
public record RobotConfig (
        String canbusName,
        SwerveConfig swerve,
        int pigeonId, IntakeConfig intake) {

    static RobotConfig createDefaultConfig() {
        return new RobotConfig(
                "",
                new SwerveConfig(
                        5.471554147,
                        3.12500,
                        5.357142857,
                        21.428571428571427,
                        new SwerveModuleConfig(1, 2, 1, 0.171631),
                        new SwerveModuleConfig(3, 4, 2, 0.454590),
                        new SwerveModuleConfig(5, 6, 3, -0.304199),
                        new SwerveModuleConfig(7, 8, 4, -0.490479)
                ),
                1,
                new IntakeConfig(9,10)

        );
    }
}

