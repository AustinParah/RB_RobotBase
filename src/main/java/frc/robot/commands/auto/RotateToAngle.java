package frc.robot.commands.auto;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;

public class RotateToAngle extends Command {
    private RotationController rotationController = Subsystems.rotationController;
    private final double targetAngleDegrees;
    private Double overrideClamp;
    private double lastScanTime = 0;
    private SwerveRequest.FieldCentric rotate = 
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final int scansToHold = 5;
    private int scanCount = 0;

    private boolean mirror = false;

    /**
     * Rotates the robot to a specific angle, assumes field mirror will be used
     * @param targetAngleDegrees
     */

    public RotateToAngle(double targetAngleDegrees) {
        this(targetAngleDegrees, true);
    }

    public RotateToAngle(double targetAngleDegrees, boolean mirror) {
        Rotation2d rotation = Rotation2d.fromDegrees(targetAngleDegrees);
        if (mirror) {
            rotation = GeometryUtil.flipFieldRotation(rotation);
        }
        this.targetAngleDegrees = rotation.getDegrees();
        rotationController.setSetpoint(targetAngleDegrees);
        addRequirements(Subsystems.swerveSubsystem);
    }

    public RotateToAngle withOverrideRadianClamp(double clamp) {
        this.overrideClamp = clamp;
        return this;
    }

    public RotateToAngle withThreshold(double threshold) {
        rotationController.setTolerance(threshold);
        return this;
    }

    @Override
    public void initialize() {
        this.lastScanTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        
        // double clamp = SmartDashboard.getNumber("AutoRotationClamp", 30);
        // if (this.overrideClamp != null) {
        //     clamp = overrideClamp;
        // }

        double twist = rotationController.calculate(
                Subsystems.swerveSubsystem.getYaw(),
                this.targetAngleDegrees);

        
        // double clampedTwist = MathUtil.clamp(Math.toRadians(twist), -clamp, clamp);

        // twist = RotationController.clampToDPS(0.2);
        final double twistRate = twist * Constants.Swerve.kMaxAngularVelocity;

        Subsystems.swerveSubsystem.setControl(rotate.withRotationalRate(twistRate));

//        System.out.println("[RotateToAngle] " + Subsystems.swerveSubsystem.getYaw() + " : " + twistRate);
//        double now = Timer.getFPGATimestamp();
//        SmartDashboard.putNumber("RotateToAngle Scan Time", (now - lastScanTime));
//        lastScanTime = now;

        if (Constants.Dashboard.UseSendables) {
            SmartDashboard.putNumber("RotateToAngle/Error", rotationController.getPositionError());
            if (Constants.Dashboard.ConfigurationMode) {
                SmartDashboard.putNumber("RotateToAngle/Twist", twist);
                SmartDashboard.putNumber("RotateToAngle/TwistRate", twistRate);
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (rotationController.atSetpoint()) {
            if (++scanCount > scansToHold) {
                return true;
            }
        } else {
            scanCount = 0;
        }
        return rotationController.atSetpoint();
    }

     @Override
     public void end(boolean interrupted) {
        rotationController.resetTolerance();
         Subsystems.swerveSubsystem.setControl(rotate.withRotationalRate(0));
     }

}
