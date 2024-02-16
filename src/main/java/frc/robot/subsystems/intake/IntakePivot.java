package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.CachedValue;
import frc.robot.subsystems.util.MotionMagicConfig;
import frc.robot.subsystems.util.PIDHelper;
import frc.robot.subsystems.util.OpenLoopSpeedsConfig;

import java.util.Objects;

public class IntakePivot implements Lifecycle, Sendable {
    private final TalonFX pivotDrive;
    private final PIDHelper pidHelper = new PIDHelper("IntakeSubsystem/Pivot/PID");
    private final TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
    private final DutyCycleOut pivotOpenLoopDriveRequest = new DutyCycleOut(0);


    private final MotionMagicConfig motionMagicConfig = new MotionMagicConfig();
    private final MotionMagicVoltage pivotMotionMagicDutyCycle = new MotionMagicVoltage(IntakePosition.Zero.setpoint);
    private final OpenLoopSpeedsConfig pivotOpenLoopSpeeds = new OpenLoopSpeedsConfig();
    //
    // Util
    //
    private final Telemetry telemetry;
    private boolean softLimitsEnabled = false;
    private boolean pivotOpenLoop = false;
    private double pivotOpenLoopSpeed = 0.0;
    private double pivotSetpoint = 0;
    private IntakePosition pivotCurrentPosition = IntakePosition.Zero;

    private double zeroPivotEncoderOffset = 24;

    public IntakePivot(TalonFX pivotDrive) {
        this.pivotDrive = pivotDrive;
        telemetry = new Telemetry();

        // kp = 0.25, ka = 0.01
        // kp 15, ka = 0.01
        pidHelper.initialize(0.25, 0, 0, 0, 0.12, 0.01);

        pivotConfiguration.SoftwareLimitSwitch
                .withForwardSoftLimitThreshold(0)
                .withReverseSoftLimitThreshold(-22);

        pidHelper.updateConfiguration(pivotConfiguration.Slot0);
        pivotConfiguration.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);

        motionMagicConfig.updateMotionMagicConfig(pivotConfiguration.MotionMagic);

        DataLogManager.log("[Pivot:IntakePivot config: " + pivotConfiguration);
        pivotDrive.getConfigurator().apply(pivotConfiguration);
    }

    @Override
    public void teleopInit() {
        holdPosition();
    }

    @Override
    public void autoInit() {
        holdPosition();
    }

    public IntakePosition getIntakePosition() {
        return pivotCurrentPosition;
    }

    public void setIntakePosition(IntakePosition newPosition) {
        pivotCurrentPosition = newPosition;
        telemetry.logPositionChange(newPosition);
    }

    public double getPivotSetpoint() {
        return this.pivotSetpoint;
    }

    public void setPivotSetpoint(double pivotSetpoint) {
        setPivotOpenLoop(false);
        this.pivotSetpoint = pivotSetpoint;
    }

    public void holdPosition() {
        double pivotSetpoint = pivotDrive.getPosition().getValue();
        setPivotSetpoint(pivotSetpoint);
    }

    public void pivotOpenLoopUp() {
        pivotOpenLoop = true;
        pivotOpenLoopSpeed = pivotOpenLoopSpeeds.getUpSpeed();
    }

    public void pivotOpenLoopDown() {
        pivotOpenLoop = true;
        pivotOpenLoopSpeed = pivotOpenLoopSpeeds.getDownSpeed();
    }

    public void pivotOpenLoopStop() {
        pivotOpenLoop = true;
        pivotOpenLoopSpeed = 0.0;
    }

    public boolean isPivotOpenLoop() {
        return pivotOpenLoop;
    }

    public void setPivotOpenLoop(boolean pivotOpenLoop) {
        this.pivotOpenLoop = pivotOpenLoop;
    }

    public void setZeroPivotEncoderOffset(double zeroPivotEncoderOffset) {
        this.zeroPivotEncoderOffset = zeroPivotEncoderOffset;
    }

    public double getZeroPivotEncoderOffset() {
        return zeroPivotEncoderOffset;
    }

    public Command getSetPivotEncoderOffestCmd() {
        return Commands.runOnce(() -> pivotDrive.setPosition(this.getZeroPivotEncoderOffset()));
    }

    public void periodic() {
        if (pivotOpenLoop) {
            pivotDrive.setControl(pivotOpenLoopDriveRequest.withOutput(pivotOpenLoopSpeed)
                    .withLimitForwardMotion(softLimitsEnabled)
                    .withLimitReverseMotion(softLimitsEnabled));
            SmartDashboard.putNumber(Intake.SUBSYSTEM_NAME + "/Pivot/RotationV", this.pivotDrive.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber(Intake.SUBSYSTEM_NAME + "/Pivot/RotationA", this.pivotDrive.getAcceleration().getValueAsDouble());
        } else {
//            if (pidHelper.updateValuesFromDashboard()) {
//                pidHelper.updateConfiguration(pivotConfiguration.Slot0);
//                pivotDrive.getConfigurator().apply(pivotConfiguration.Slot0);
//            }
            pivotDrive.setControl(pivotMotionMagicDutyCycle.withPosition(pivotSetpoint)
                    .withLimitForwardMotion(softLimitsEnabled)
                    .withLimitReverseMotion(softLimitsEnabled));
        }
    }

    public void updatePIDFromDashboard() {
        pidHelper.updateConfiguration(pivotConfiguration.Slot0);
        pivotConfiguration.Slot0
                .withKG(motionMagicConfig.kG)
                .withKS(motionMagicConfig.kS);
        motionMagicConfig.updateMotionMagicConfig(pivotConfiguration.MotionMagic);
        StatusCode result = pivotDrive.getConfigurator().apply(pivotConfiguration);
        DataLogManager.log("[IntakePivot] update PID info from dash: " + pivotConfiguration + ": result = " + result);
    }

    public Command updatePIDFromDashbboardCommand() {
        return Commands.runOnce(this::updatePIDFromDashboard);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Pivot/OpenLoop", this::isPivotOpenLoop, this::setPivotOpenLoop);
        builder.addDoubleProperty("Pivot/UpSpeed", pivotOpenLoopSpeeds::getUpSpeed, pivotOpenLoopSpeeds::setUpSpeed);
        builder.addDoubleProperty("Pivot/DownSpeed", pivotOpenLoopSpeeds::getDownSpeed, pivotOpenLoopSpeeds::setDownSpeed);

        builder.addDoubleProperty("Pivot/Setpoint", this::getPivotSetpoint, this::setPivotSetpoint);
        builder.addStringProperty("Pivot/Position", () -> this.getIntakePosition().name(), null);

        builder.addDoubleProperty("Pivot/MM/kS", motionMagicConfig::getkS, motionMagicConfig::setkS);
        builder.addDoubleProperty("Pivot/MM/kG", motionMagicConfig::getkG, motionMagicConfig::setkG);
        builder.addDoubleProperty("Pivot/MM/Acceleration", motionMagicConfig::getAcceleration, motionMagicConfig::setAcceleration);
        builder.addDoubleProperty("Pivot/MM/Velocity", motionMagicConfig::getVelocity, motionMagicConfig::setVelocity);
        builder.addDoubleProperty("Pivot/MM/Jerk", motionMagicConfig::getJerk, motionMagicConfig::setJerk);

        builder.addBooleanProperty("Pivot/FwdLimitHit", () -> this.pivotDrive.getFault_ForwardSoftLimit().getValue(), null);
        builder.addBooleanProperty("Pivot/RevLimitHit", () -> this.pivotDrive.getFault_ReverseSoftLimit().getValue(), null);

        builder.addDoubleProperty("Pivot/Encoder", () -> this.pivotDrive.getPosition().getValue(), null);
        builder.addDoubleProperty("Pivot/ZeroEncoderOffset", this::getZeroPivotEncoderOffset, this::setZeroPivotEncoderOffset);
    }


    public enum IntakePosition {
        Zero(0),
        Passing(24), // TODO: WILD GUESS
        Pickup(20), // TODO: WILD GUESS
        AMP(10);     // TODO: WILD GUESS

        private final int setpoint;

        IntakePosition(int setpoint) {
            this.setpoint = setpoint;
        }
    }

    class Telemetry {
        private final StringLogEntry pivotPositionLog;
        CachedValue<IntakePosition> positionCache = new CachedValue<>(pivotCurrentPosition, Objects::equals);

        Telemetry() {
            DataLog log = DataLogManager.getLog();
            pivotPositionLog = new StringLogEntry(log, "Intake/PivotPosition");
        }

        void logPositionChange(IntakePosition position) {
            if (positionCache.update(position)) {
                pivotPositionLog.append(position.name());
            }
        }
    }
}