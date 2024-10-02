package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase implements Lifecycle, Sendable {
    private final TalonFX motor = new TalonFX(54);
    private final DutyCycleOut openLoopOut = new DutyCycleOut(0);

    public Feeder() {
    }

    @Override
    public void teleopInit() {
        updateFeeder(0);
    }

    public void updateFeeder(double speed) {
        motor.setControl(openLoopOut.withOutput(speed));
    }

    public Command runFeederCmd() {
        return Commands.run(() -> this.updateFeeder(1.0), this);
    }

    public Command stopFeederCmd() {
        return Commands.run(() -> this.updateFeeder(0.0), this);
    }
}
