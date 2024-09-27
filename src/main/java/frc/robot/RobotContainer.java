package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.util.GameInfo;

public class RobotContainer {
  boolean isRedAlliance;
  double MaxSpeed = 1.0;
  double MaxAngularRate = 1.0;


  // Baisc Stuff
  private final Joystick left = new Joystick(0);
  private final Joystick right = new Joystick(1);
  
  private final Subsystems subsystems = Subsystems.getInstance();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  //Intake
  private final JoystickButton grabNote = new JoystickButton(left, 1);
  private final JoystickButton shootNote = new JoystickButton(right, 1);

  


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

  public void autoInit() {
    
  }

  public void teleopInit() {

  }

  private void configureBindings(){


  }

  private void configureDashboardButtons(){
    
  }
}