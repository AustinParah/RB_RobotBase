package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.config.RobotConfiguration;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.GameInfo;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;


import static edu.wpi.first.units.Units.RadiansPerSecond;

public class RobotContainer {
  boolean isRedAlliance;
  double MaxSpeed = Constants.Swerve.kMaxSpeedMetersPerSecond;
  double MaxAngularRate = Constants.Swerve.kMaxAngularVelocity;


  // Baisc Stuff
  private final Joystick left = new Joystick(0);
  private final Joystick right = new Joystick(1);
  
  private final Subsystems subsystems = Subsystems.getInstance();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  //Intake
  private final JoystickButton grabNote = new JoystickButton(left, 1);
  private final JoystickButton shootNote = new JoystickButton(right, 1);
  private final JoystickButton cancelGrab = new JoystickButton(left, 2);

  private final JoystickButton startShooter = new JoystickButton(right, 2);
  private final JoystickButton stopShooter = new JoystickButton(right, 4);


  public void robotPeriodic() {
    Subsystems.shooterSubsystem.periodic();

    if(Subsystems.feederSubsystem.isNotedDetected() && Subsystems.intakeSubsystem.isRunning && Subsystems.intakeSubsystem.isAutointake){
      Subsystems.intakeSubsystem.isAutointake = false;
      Subsystems.feederSubsystem.stopFeederCmd();
      Subsystems.intakeSubsystem.stopIntakeCmd();
    }
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
    final double DEADBAND = 0.05;
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

    // TODO: Add button if we want this
    /*robotCentric.whileTrue(
            Subsystems.swerveSubsystem.applyRequest(() -> robotCentricDrive

                    .withDeadband(0.02 * MaxSpeed)
                    .withVelocityX(OIUtil.deadband(supplySwerveY(), 0.05) * MaxSpeed)
                    .withVelocityY(OIUtil.deadband(supplySwerveY(), 0.05) * MaxSpeed)
                    .withRotationalRate(supplySwerveRotate().in(RadiansPerSecond))));*/

    //
    // Commands
    //
    //Jason implementation
    grabNote.onTrue(
      Subsystems.intakeSubsystem.runIntakeFastCmd()
              .alongWith(
                      Subsystems.feederSubsystem.runFeederCmd()
              )).and(() -> !Subsystems.feederSubsystem.isNotedDetected())
      .onFalse(
              Subsystems.intakeSubsystem.stopIntakeCmd()
                      .alongWith(
                              Subsystems.feederSubsystem.stopFeederCmd()));

  // ChatGPT implementation
  grabNote.onTrue(
    new ConditionalCommand(
        Subsystems.intakeSubsystem.runIntakeFastCmd()
            .alongWith(Subsystems.feederSubsystem.runFeederCmd()), 
        Subsystems.intakeSubsystem.stopIntakeCmd()
            .alongWith(Subsystems.feederSubsystem.stopFeederCmd()),
        Subsystems.feederSubsystem::isNoteDetected // Automatically stops when note is detected
    )
).onFalse(
    Subsystems.intakeSubsystem.stopIntakeCmd()
        .alongWith(Subsystems.feederSubsystem.stopFeederCmd())
);

// grabNote.onTrue(
//   new ParallelCommandGroup(
//       Subsystems.intakeSubsystem.runIntakeFastCmd(),
//       Subsystems.feederSubsystem.runFeederCmd()
//   )
// ).onFalse(
//   new InstantCommand(() -> {
//       Subsystems.intakeSubsystem.stopIntakeCmd();
//       Subsystems.feederSubsystem.stopFeederCmd();
//   })
// );

// // Override automatic stopping when the note is detected
// Subsystems.feederSubsystem.setDefaultCommand(
//   new ConditionalCommand(
//       new InstantCommand(() -> {
//           // Stop intake and feeder automatically if the note is detected
//           Subsystems.intakeSubsystem.stopIntakeCmd();
//           Subsystems.feederSubsystem.stopFeederCmd();
//       }),
//       new ParallelCommandGroup(
//           Subsystems.intakeSubsystem.runIntakeFastCmd(),
//           Subsystems.feederSubsystem.runFeederCmd()
//       ),
//       Subsystems.feederSubsystem::isNoteDetected
//   )
// );


    //grabNote.onTrue(new InstantCommand(this::handleIntakeLogic));
    // if(!Subsystems.intakeSubsystem.isRunning){ // todo: ignore my spaghetti
    //   grabNote.onTrue( ParallelCommandGroup(
    //   Subsystems.intakeSubsystem.runIntakeFastCmd()),
    //   Subsystems.feederSubsystem.runFeederCmd());
    // } else{
    // grabNote.onTrue( Subsystems.intakeSubsystem.stopIntakeCmd()
    //                         .alongWith(Subsystems.feederSubsystem.stopFeederCmd()));
    // }

    //NOTE: TEST CASE
    // grabNote.onTrue(
    //   new InstantCommand(() -> Subsystems.intakeSubsystem.setOutput(5.0))
    // );

    shootNote.onTrue(Subsystems.feederSubsystem.shootFeederCmd()).onFalse(Subsystems.feederSubsystem.stopFeederCmd());

    startShooter.onTrue(Subsystems.shooterSubsystem.startShooterCmd());
    stopShooter.onTrue(Subsystems.shooterSubsystem.stopShooterCmd());
  }

  private void configureDashboardButtons(){
    
  }

  private void handleIntakeLogic(){
    if(!Subsystems.intakeSubsystem.isRunning){
      Subsystems.intakeSubsystem.runIntakeFastCmd();
      Subsystems.feederSubsystem.runFeederCmd();
      Subsystems.intakeSubsystem.isAutointake = true;
      return;
    }
    Subsystems.intakeSubsystem.stopIntakeCmd();
    Subsystems.feederSubsystem.stopFeederCmd();
    Subsystems.intakeSubsystem.isAutointake = false;
  }
}