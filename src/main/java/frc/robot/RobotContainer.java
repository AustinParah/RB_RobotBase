// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ShooterPrototype;

public class RobotContainer {
  private static final double MaxSpeed = Constants.Swerve.kMaxSpeedMetersPerSecond;
  private static final double MaxAngularRate = Constants.Swerve.kMaxAngularVelocity;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final Joystick left = new Joystick(0);
  private final Joystick right = new Joystick(1);
  private final CommandXboxController xboxController = new CommandXboxController(2); 
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; 
  private Subsystems subsystems = Subsystems.getInstance();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final Trigger robotCentric = new Trigger(left::getTrigger);

  private final Trigger shooterPrototypeOpenLoop = xboxController.x();
  private final Trigger shooterPrototypeClosedLoop = xboxController.a();
  
  

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(-right.getY() * MaxSpeed)
            .withVelocityY(-right.getX() * MaxSpeed)
            .withRotationalRate(-left.getX() * MaxAngularRate)));

    xboxController.y().whileTrue(drivetrain.applyRequest(() -> brake));
    xboxController.b().whileTrue(drivetrain
        .applyRequest(
            () -> point.withModuleDirection(new Rotation2d(-xboxController.getLeftY(), -xboxController.getLeftX()))));

    shooterPrototypeOpenLoop.onTrue(new InstantCommand(Subsystems.shooterPrototype::toggleOpenLoop));
    shooterPrototypeClosedLoop.onTrue(new InstantCommand(Subsystems.shooterPrototype::toggleClosedLoop));


    // reset the field-centric heading on left bumper press
    robotCentric.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void teleopInit() {
    Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.teleopInit());
  }

  public void autoInit() {
    Subsystems.lifecycleSubsystems.stream().filter(s -> s != null).forEach((s) -> s.autoInit());
  }

}
