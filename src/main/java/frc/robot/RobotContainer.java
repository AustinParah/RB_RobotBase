package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;


import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.GameInfo;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public class RobotContainer {
  boolean isRedAlliance;
  double MaxSpeed = Constants.Swerve.kMaxSpeedMetersPerSecond;
  double MaxAngularRate = Constants.Swerve.kMaxAngularVelocity;


  // Baisc Stuff
  private final Joystick left = new Joystick(0);
  private final Joystick right = new Joystick(1);

  private final CommandXboxController gamepad = new CommandXboxController(3);
  
  private final Subsystems subsystems = Subsystems.getInstance();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);



  public void robotPeriodic() {

  }

  public Command getAutonomousCommand() {
    return Subsystems.autoManager.getSelectedAutoStrategy();
  }

  public RobotContainer() {
    configureBindings();
    configureDashboardButtons();
    SmartDashboard.setDefaultNumber("AlignPIDFactor", 200);

    updateRedAllianceInfo();
  }

  public void updateRedAllianceInfo() {
    isRedAlliance = GameInfo.isRedAlliance();
    //SLogger.log("RobotContainer", "updateRedAllianceInfo = " + isRedAlliance);
  }

  public double supplySwerveX() {
    double base = -right.getY();
    return (isRedAlliance) ? -1 * base : base;
  }

  public double supplySwerveY() {
    double base = -right.getX();
    return isRedAlliance ? -1 * base : base;
  }

  /**
   * This method is used to supply the swerve rotate value to the swerve drive
   *
   * @return the swerve rotate value
   */
  private Measure<Velocity<Angle>> supplySwerveRotate() {
    final double DEADBAND = 0.1;
    final double twist;
    twist = OIUtil.deadband(-left.getX(), DEADBAND) * (MaxAngularRate * 0.8);
    return RadiansPerSecond.of(twist);
  }

  public void autoInit() {
    Subsystems.lifecycleSubsystems.forEach(Lifecycle::autoInit);
  }

  public void teleopInit() {
    Subsystems.lifecycleSubsystems.forEach(Lifecycle::teleopInit);
  }

  private void configureBindings(){
    Subsystems.swerveSubsystem.setDefaultCommand( // Drivetrain will execute this command periodically
            Subsystems.swerveSubsystem.applyRequest(() -> drive
                    .withDeadband(0.02 * MaxSpeed)
                    .withVelocityX(OIUtil.deadband(supplySwerveX(), 0.05) * MaxSpeed)
                    .withVelocityY(OIUtil.deadband(supplySwerveY(), 0.05) * MaxSpeed)
                    .withRotationalRate(supplySwerveRotate().in(RadiansPerSecond))));
  }

  private void configureDashboardButtons(){
    
  }
}