package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.Intake;

/**
 * Global subsystem declaration and tracking to allow easier injection
 * for commands
 */
public class Subsystems {
    private static Subsystems instance;
    public static CommandSwerveDrivetrain swerveSubsystem;
    public static VisionSubsystem visionSubsystem;
    public static LEDSubsystem ledSubsystem;
    public static Intake intake;

    public static Shooter shooter;

    // Utility
    public static RotationController rotationController = new RotationController();

    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();

    private Subsystems() {
        swerveSubsystem = TunerConstants.DriveTrain;
        visionSubsystem = new VisionSubsystem();
        ledSubsystem = new LEDSubsystem();
        intake = new Intake();
        shooter = new Shooter();
        
        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);
        lifecycleSubsystems.add(intake);
        lifecycleSubsystems.add(shooter);

        SmartDashboard.putData("ShooterSubsystem", shooter);
    }


    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
