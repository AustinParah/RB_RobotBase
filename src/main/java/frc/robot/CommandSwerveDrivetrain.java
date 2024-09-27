package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DMS.DriveInfo;

import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    public final SwerveRequest DMSDriveRequest = new SwerveRequest() {
        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            for (int i = 0; i < modulesToApply.length; ++i) {
                SwerveModuleState state = new SwerveModuleState(0.5 * Constants.Swerve.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(0.0));
                modulesToApply[i].apply(state, SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
            }
            return StatusCode.OK;
        }
    };
    public final SwerveRequest DMSSteerRequest = new SwerveRequest() {
        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            for (int i = 0; i < modulesToApply.length; ++i) {
                double currentAngle = modulesToApply[i].getCurrentState().angle.getDegrees();
                double targetAngle = (currentAngle + ((4 * (360.0 / 50.0)) % 360.0));

                SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(targetAngle));
                modulesToApply[i].apply(state, SwerveModule.DriveRequestType.OpenLoopVoltage, SwerveModule.SteerRequestType.MotionMagic);
            }
            return StatusCode.OK;
        }
    };
    // Internal command used for path planning auto
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
                                   SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        postConstructConfig();
        //configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        postConstructConfig();
        //configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void postConstructConfig() {
        this.configNeutralMode(NeutralModeValue.Coast);
        for (int i = 0; i < this.ModuleCount; i++) {
            var module = this.getModule(i);
            module.getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
            module.getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
        }
    }

//     private void configurePathPlanner() {
//         double driveBaseRadius = 0.0;
//         for (var moduleLocation : m_moduleLocations) {
//             driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
//         }

//         // Assume Blue is default
//         AutoBuilder.configureHolonomic(
//                 () -> this.getState().Pose,
//                 this::seedFieldRelative,
//                 this::getCurrentRobotChassisSpeeds,
//                 (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
//                 new HolonomicPathFollowerConfig(
//                         new PIDConstants(6, 0, 0),
//                         new PIDConstants(10, 0, 0),
//                         Constants.AutoConstants.kMaxSpeedMetersPerSecond, // TunerConstants.kSpeedAtVoltsMps,
//                         driveBaseRadius,
//                         new ReplanningConfig()),
//  //               GameInfo::isRedAlliance,
//                 this);

//     }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Get the path planner auto command for a named path. Deprecated, use Subsystems.autoManager.getAutoPath instead.
     *
     * @param pathName the name of the path to run
     * @return a Command that will run the named path
     */
    @Deprecated
    public Command getAutoPath(String pathName) {
        try {
            return Subsystems.autoManager.getAutoPath(pathName);
        } catch (Exception e) {
           // BSLogger.log("CommandServeDrivetrain", "!!! Error attempting to locate auto path %s: %s".formatted(pathName, e));
            return new PrintCommand("Cannot Locate path" + pathName);
        }
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public double getYaw() {
        return this.getState().Pose.getRotation().getDegrees();
        // return this.getPigeon2().getYaw().getValueAsDouble() % 360;
    }

    public DriveInfo<Double> getDriveOutputCurrent() {
        return new DriveInfo<Double>(
                this.getModule(0).getDriveMotor().getSupplyCurrent().getValue(),
                this.getModule(1).getDriveMotor().getSupplyCurrent().getValue(),
                this.getModule(2).getDriveMotor().getSupplyCurrent().getValue(),
                this.getModule(3).getDriveMotor().getSupplyCurrent().getValue()
        );
    }

    public DriveInfo<Double> getDriveVelocity() {
        return new DriveInfo<Double>(
                this.getModule(0).getDriveMotor().getVelocity().getValue(),
                this.getModule(1).getDriveMotor().getVelocity().getValue(),
                this.getModule(2).getDriveMotor().getVelocity().getValue(),
                this.getModule(3).getDriveMotor().getVelocity().getValue()
        );
    }

    public DriveInfo<Double> getSteerOutputCurrent() {
        return new DriveInfo<Double>(
                this.getModule(0).getSteerMotor().getSupplyCurrent().getValue(),
                this.getModule(1).getSteerMotor().getSupplyCurrent().getValue(),
                this.getModule(2).getSteerMotor().getSupplyCurrent().getValue(),
                this.getModule(3).getSteerMotor().getSupplyCurrent().getValue()
        );
    }

    public DriveInfo<Double> getSteerVelocity() {
        return new DriveInfo<Double>(
                this.getModule(0).getSteerMotor().getVelocity().getValue(),
                this.getModule(1).getSteerMotor().getVelocity().getValue(),
                this.getModule(2).getSteerMotor().getVelocity().getValue(),
                this.getModule(3).getSteerMotor().getVelocity().getValue()
        );
    }

}
