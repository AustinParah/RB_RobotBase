// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.RobotConfiguration;
import frc.robot.config.BuildConstants;


public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;


  @Override
  public void robotInit() {
//    DataLogManager.start();
    RobotConfiguration.initialize();
    robotContainer = new RobotContainer();

    Subsystems.swerveSubsystem.getPigeon2().setYaw(0);

//    for (int i=0;i<4;i++) {
//      Subsystems.swerveSubsystem.getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
//    }


    // Forward LimeLight ports so they are available over USB
//    for (int port = 5800; port <= 5807; port++) {
//        PortForwarder.add(port, "limelight.local", port);
//    }


 //   addPeriodic(Subsystems.ledSubsystem::Report, 0.1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.robotPeriodic();
//    powerTelemetry.periodic();
    for (int i=0;i<4;i++) {
      var module = Subsystems.swerveSubsystem.getModule(i);
      SmartDashboard.putNumber("Module" + i + " Abs", module.getCANcoder().getAbsolutePosition().getValue());
      SmartDashboard.putNumber("Module" + i + " Pos", module.getCANcoder().getPosition().getValue());
      SmartDashboard.putData(Subsystems.swerveSubsystem.getModule(i).getCANcoder());
    }
  }


  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    robotContainer.autoInit();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
//      BSLogger.log("Robot", "autoInit:: scheduled command at: " + Timer.getFPGATimestamp());
    }
  //  BSLogger.log("Robot", "autoInit:: finished at: " + Timer.getFPGATimestamp());
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    for (int i=0;i<4;i++) {
      Subsystems.swerveSubsystem.getModule(i).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
