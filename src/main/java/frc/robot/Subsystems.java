package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.async.AsyncManager;
import frc.robot.auto.AutoManager;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

/**
 * The Subsystems class represents a collection of subsystems used in the robot.
 * It provides singleton access to all the subsystem instances and manages their
 * lifecycle.
 */
public class Subsystems {
    public static CommandSwerveDrivetrain swerveSubsystem;

    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();

    //
    // Utility classes
    //
    public static AsyncManager asyncManager;

    // Warning: This must be created after everything else to ensure all subsystems
    // are registered
    public static AutoManager autoManager;

    private static Subsystems instance;
    public static Intake intakeSubsystem;
    public static Shooter shooterSubsystem;
    public static Feeder feederSubsystem;

    public Subsystems() {
        swerveSubsystem = TunerConstants.DriveTrain;
        intakeSubsystem = new Intake();
        shooterSubsystem = new Shooter();
        feederSubsystem = new Feeder();
        createUtilitySubsystems();

        SmartDashboard.putData(CommandScheduler.getInstance());

        SmartDashboard.putData("IntakeSubsystem", intakeSubsystem);
        SmartDashboard.putData("ShooterSubsystem", shooterSubsystem);
        SmartDashboard.putData("FeederSubsystem", feederSubsystem);
    }

    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }

    private void createUtilitySubsystems() {
        asyncManager = new AsyncManager();
        asyncManager.start();

        autoManager = new AutoManager();
        autoManager.initialize();
    }
}
