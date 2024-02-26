package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.trap.Trap;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionTypes;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;


/**
 * The Subsystems class represents a collection of subsystems used in the robot.
 * It provides singleton access to all the subsystem instances and manages their lifecycle.
 */
@SuppressWarnings("InstantiationOfUtilityClass")
public class Subsystems {
    public static CommandSwerveDrivetrain swerveSubsystem;
    public static VisionSubsystem visionSubsystem;
    public static LEDSubsystem ledSubsystem;
    public static Intake intake;
    public static Shooter shooter;
    public static Pivot pivot;
    public static Climber climber;
    public static Trap trap;

    // Utility
    public static PoseManager poseManager;

    public static RotationController rotationController = new RotationController();
    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();
    private static Subsystems instance;

    private Subsystems() {
        swerveSubsystem = TunerConstants.DriveTrain;
        visionSubsystem = new VisionSubsystem(
                Stream.of(
                        // was 12.92 degrees
//                        new VisionTypes.LimelightInfo("limelight", (11.57625659 - (24.84 * 2)) , 24.84)),
//                            new VisionTypes.LimelightInfo("limelight", 12.92 , 25.71    ))
                                new VisionTypes.LimelightInfo("limelight", 13.1 , 25.71    ))

//                        new VisionTypes.LimelightInfo("notelight", 10, 0))
                        .map(Limelight::new).collect(Collectors.toSet()));
        ledSubsystem = new LEDSubsystem();
        //TODO add if to prevent comp bot subsystem calls on practice bot
        intake = new Intake();
        shooter = new Shooter();
        pivot = new Pivot();
        climber = new Climber();
//        trap = new Trap();

        poseManager = new PoseManager();


        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);
        lifecycleSubsystems.add(intake);
        lifecycleSubsystems.add(shooter);
        lifecycleSubsystems.add(pivot);
        lifecycleSubsystems.add(climber);
//        lifecycleSubsystems.add(trap);

       SmartDashboard.putData("ShooterSubsystem", shooter);
       SmartDashboard.putData("PivotSubsystem", pivot);
       SmartDashboard.putData("IntakeSubsystem", intake);
       SmartDashboard.putData("ClimberSubsystem", climber);
//       SmartDashboard.putData("TrapSubsystem", trap);
    }


    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
