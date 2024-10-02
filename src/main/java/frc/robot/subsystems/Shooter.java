package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements Lifecycle, Sendable {
    private final ShooterHelper upper = new ShooterHelper("ShooterSubsystem", "Upper", new TalonFX(50));
    private final ShooterHelper lower = new ShooterHelper("ShooterSubsystem", "Lower", new TalonFX(55));

    public Shooter() {
    }

    @Override
    public void periodic() {
        upper.periodic();
        lower.periodic();
    }

    public Command startShooterCmd() {
        return Commands.runOnce(() -> {
            upper.setEnabled(true);
            lower.setEnabled(true);
            upper.setOpenLoop(false);
            lower.setOpenLoop(false);
            upper.setVelocitySetpoint(-50);
            lower.setVelocitySetpoint(-50);
//            upper.periodic();
//            lower.periodic();
        }, this);
    }

    public Command stopShooterCmd() {
        return Commands.runOnce(() -> {
            upper.setVelocitySetpoint(0);
            lower.setVelocitySetpoint(0);
//            upper.periodic();
//            lower.periodic();
            upper.setEnabled(false);
            lower.setEnabled(false);
        }, this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("ShooterSubsystem");
        upper.initSendable(builder);
        lower.initSendable(builder);
    }
}
