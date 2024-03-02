package frc.robot.subsystems.trap;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.BSLogger;
import frc.robot.subsystems.util.MotionMagicConfig;
import frc.robot.subsystems.util.OpenLoopSpeedsConfig;
import frc.robot.subsystems.util.PIDHelper;

public class TrapPivot implements Lifecycle, Sendable {
    private final TalonFX motor;
    private final TalonFXConfiguration configuration = new TalonFXConfiguration();
    private final OpenLoopSpeedsConfig openLoopSpeeds = new OpenLoopSpeedsConfig(0.1, -0.1);
    private final MotionMagicConfig motionMagicConfig = new MotionMagicConfig();
    private final DutyCycleOut openLoopOut = new DutyCycleOut(0);
    private final PositionVoltage positionOut = new PositionVoltage(0);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private final PIDHelper pidHelper = new PIDHelper(Trap.SUBSYSTEM_NAME + "/Pivot/PID");
    private boolean openLoop = true;
    private double openLoopSetpoint = 0;
    private double closedLoopSetpoint;
    private TrapPivotPosition trapPosition = TrapPivotPosition.Zero;

    public enum TrapPivotPosition {
        Zero(0),
        Feed(0);

        private final double setpoint;

        TrapPivotPosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    public TrapPivot(TalonFX motor) {
        this.motor = motor;
        pidHelper.initialize(0, 0, 0, 0, 0, 0);
        updatePIDFromDashboard();
        motor.getConfigurator().apply(configuration);
    }

    public boolean isOpenLoop() {
        return openLoop;
    }

    public void setOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
    }

    public void openLoopUp() {
        setOpenLoop(true);
        setOpenLoopSetpoint(openLoopSpeeds.getUpSpeed());
    }

    public void openLoopDown() {
        setOpenLoop(true);
        setOpenLoopSetpoint(openLoopSpeeds.getDownSpeed());
    }

    public void stopOpenLoop() {
        setOpenLoop(true);
        setOpenLoopSetpoint(0.0);
    }

    public double getOpenLoopSetpoint() {
        return openLoopSetpoint;
    }

    public void setOpenLoopSetpoint(double openLoopSetpoint) {
        this.openLoopSetpoint = openLoopSetpoint;
    }

    public double getClosedLoopSetpoint() {
        return closedLoopSetpoint;
    }

    public void setClosedLoopSetpoint(double closedLoopSetpoint) {
        this.closedLoopSetpoint = closedLoopSetpoint;
    }

    public void setTrapPosition(TrapPivotPosition trapPosition) {
        this.trapPosition = trapPosition;
        this.openLoop = false;
        this.setClosedLoopSetpoint(trapPosition.setpoint);
    }

    public void updatePIDFromDashboard() {
        pidHelper.updateConfiguration(configuration.Slot0);
        motionMagicConfig.updateSlot0Config(configuration.Slot0);
        motionMagicConfig.updateMotionMagicConfig(configuration.MotionMagic);
        StatusCode result = motor.getConfigurator().apply(configuration);
        if (result != StatusCode.OK) {
            BSLogger.log("TrapPivot", "Failed to apply configuration to motor: " + result);
        }
    }

    public Command updateClosedLoopFromDashbboardCommand() {
        return Commands.runOnce(this::updatePIDFromDashboard);
    }


    public void periodic() {
//        updatePIDFromDashboard();
        if (openLoop) {
            motor.setControl(openLoopOut.withOutput(openLoopSetpoint));
        } else {
            motor.setControl(positionOut.withPosition(closedLoopSetpoint));
//            motor.setControl(motionMagicVoltage.withPosition(closedLoopSetpoint));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("TrapPivot/Encoder", () -> this.motor.getPosition().getValue(), null);
        if (Constants.Dashboard.ConfigurationMode || Constants.Dashboard.TrapConfigMode) {

            builder.addBooleanProperty("TrapPivot/OpenLoop", this::isOpenLoop, this::setOpenLoop);
            builder.addDoubleProperty("TrapPivot/UpSpeed", openLoopSpeeds::getUpSpeed, openLoopSpeeds::setUpSpeed);
            builder.addDoubleProperty("TrapPivot/DownSpeed", openLoopSpeeds::getDownSpeed, openLoopSpeeds::setDownSpeed);

            builder.addDoubleProperty("TrapPivot/ClosedLoopSetpoint", this::getClosedLoopSetpoint, this::setClosedLoopSetpoint);
            
            builder.addDoubleProperty("TrapPivot/MM/kS", motionMagicConfig::getkS, motionMagicConfig::setkS);
            builder.addDoubleProperty("TrapPivot/MM/kG", motionMagicConfig::getkG, motionMagicConfig::setkG);
            builder.addDoubleProperty("TrapPivot/MM/Acceleration", motionMagicConfig::getAcceleration, motionMagicConfig::setAcceleration);
            builder.addDoubleProperty("TrapPivot/MM/Velocity", motionMagicConfig::getVelocity, motionMagicConfig::setVelocity);
            builder.addDoubleProperty("TrapPivot/MM/Jerk", motionMagicConfig::getJerk, motionMagicConfig::setJerk);

            builder.addBooleanProperty("TrapPivot/FwdLimitHit", () -> this.motor.getFault_ForwardSoftLimit().getValue(), null);
            builder.addBooleanProperty("TrapPivot/RevLimitHit", () -> this.motor.getFault_ReverseSoftLimit().getValue(), null);
        }
//        builder.addDoubleProperty("TrapPivot/ZeroEncoderOffset", this::getZeroPivotEncoderOffset, this::setZeroPivotEncoderOffset);
    }
}
