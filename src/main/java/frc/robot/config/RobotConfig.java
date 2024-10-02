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
                        3.5714285714285716,
                        6.122448979591837,
                        21.428571428571427,
                        new SwerveModuleConfig(1, 2, 1, -0.224853515625),
                        new SwerveModuleConfig(3, 4, 2, -0.099576171875),
                        new SwerveModuleConfig(5, 6, 3, 0.126220703125),
                        new SwerveModuleConfig(7, 8, 4,  -0.148681640625)
                ),
                1,
                new IntakeConfig(9,10)

        );
    }
}

