package frc.robot.subsystems.trap;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.util.MotionMagicConfig;
import frc.robot.subsystems.util.OpenLoopSpeedsConfig;
import frc.robot.subsystems.util.PIDHelper;

public class TrapExtender implements Lifecycle, Sendable {
    private final TalonFX motor;
    private final TalonFXConfiguration configuration;
    private final OpenLoopSpeedsConfig openLoopSpeeds = new OpenLoopSpeedsConfig(-0.3, 0.3);
    private final MotionMagicConfig motionMagicConfig = new MotionMagicConfig();
    private final DutyCycleOut openLoopOut = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private final PIDHelper pidHelper = new PIDHelper(Trap.SUBSYSTEM_NAME + "/TrapExtender/PID");
    private boolean openLoop = true;
    private double openLoopSetpoint = 0;
    private double closedLoopSetpoint;

    private TrapPosition trapPosition = TrapPosition.Zero;


    public TrapExtender(TalonFX motor) {
        this.motor = motor;
        configuration = new TalonFXConfiguration()
                .withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withReverseSoftLimitThreshold(-24.25).withReverseSoftLimitEnable(true)   // 0.25 = 1/8"
                                .withForwardSoftLimitThreshold(0.55).withForwardSoftLimitEnable(true)
                ).withHardwareLimitSwitch(new HardwareLimitSwitchConfigs().withForwardLimitEnable(false).withReverseLimitEnable(false));
        pidHelper.initialize(2.6, 0.6, 0, 0, 0, 0);
        motionMagicConfig.setJerk(2000);
        motionMagicConfig.setVelocity(60);
        motionMagicConfig.setAcceleration(300);
        updatePIDFromDashboard();
        pidHelper.updateConfiguration(configuration.Slot0);
        motionMagicConfig.updateSlot0Config(configuration.Slot0);
        motor.getConfigurator().apply(configuration);
    }

    @Override
    public void teleopInit() {
        this.setOpenLoop(true);
        this.closedLoopSetpoint = motor.getPosition().getValue();
    }

    @Override
    public void autoInit() {
        this.setOpenLoop(true);
        this.closedLoopSetpoint = motor.getPosition().getValue();
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
        this.setOpenLoop(false);
        this.closedLoopSetpoint = closedLoopSetpoint;
    }

    public void setTrapPosition(TrapPosition trapPosition) {
        this.trapPosition = trapPosition;
        this.setClosedLoopSetpoint(trapPosition.setpoint);
    }

    public void updatePIDFromDashboard() {
        pidHelper.updateConfiguration(configuration.Slot0);
        motionMagicConfig.updateSlot0Config(configuration.Slot0);
        motionMagicConfig.updateMotionMagicConfig(configuration.MotionMagic);
        StatusCode result = motor.getConfigurator().apply(configuration);
        DataLogManager.log("[TrapExtender] update PID info from dash: " + configuration + ": result = " + result);
    }

    public Command updateClosedLoopFromDashbboardCommand() {
        return Commands.runOnce(this::updatePIDFromDashboard);
    }


    public void periodic() {
        if (openLoop) {
            motor.setControl(openLoopOut.withOutput(openLoopSetpoint));
        } else {
//            updatePIDFromDashboard();
            motor.setControl(motionMagicVoltage.withPosition(closedLoopSetpoint));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("TrapExtender/OpenLoop", this::isOpenLoop, this::setOpenLoop);
        builder.addDoubleProperty("TrapExtender/ClosedLoopSetpoint", this::getClosedLoopSetpoint, this::setClosedLoopSetpoint);
        builder.addStringProperty("TrapExtender/Position", () -> this.trapPosition.name(), null);

        if (Constants.Dashboard.ConfigurationMode) {
            builder.addDoubleProperty("TrapExtender/UpSpeed", openLoopSpeeds::getUpSpeed, openLoopSpeeds::setUpSpeed);
            builder.addDoubleProperty("TrapExtender/DownSpeed", openLoopSpeeds::getDownSpeed, openLoopSpeeds::setDownSpeed);

            builder.addDoubleProperty("TrapExtender/MM/kS", motionMagicConfig::getkS, motionMagicConfig::setkS);
            builder.addDoubleProperty("TrapExtender/MM/kG", motionMagicConfig::getkG, motionMagicConfig::setkG);
            builder.addDoubleProperty("TrapExtender/MM/Acceleration", motionMagicConfig::getAcceleration, motionMagicConfig::setAcceleration);
            builder.addDoubleProperty("TrapExtender/MM/Velocity", motionMagicConfig::getVelocity, motionMagicConfig::setVelocity);
            builder.addDoubleProperty("TrapExtender/MM/Jerk", motionMagicConfig::getJerk, motionMagicConfig::setJerk);

            builder.addBooleanProperty("TrapExtender/FwdLimitHit", () -> this.motor.getFault_ForwardSoftLimit().getValue(), null);
            builder.addBooleanProperty("TrapExtender/RevLimitHit", () -> this.motor.getFault_ReverseSoftLimit().getValue(), null);

            builder.addDoubleProperty("TrapExtender/Encoder", () -> this.motor.getPosition().getValue(), null);
        }
    }

    public enum TrapPosition {
        Zero(0.35),
        Up(-23.5),
        Middle(-12.0);

        private final double setpoint;

        TrapPosition(double setpoint) {
            this.setpoint = setpoint;
        }
    }

}
