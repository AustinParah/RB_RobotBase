package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements Lifecycle, Sendable {
    private final TalonFX intakeDrive;
    private final VoltageOut intakeDrive_Request = new VoltageOut(0.0);
    public boolean isRunning = false;
    public boolean isAutointake = false; // NOTE: DONT EDIT IN FILE, HANDLED BY HANDLER IN CONTAINER


    public Intake() {
        this.intakeDrive = new TalonFX(59);
    }

    @Override
    public void teleopInit() {
        setOutput(0);
    }

    public void setOutput(double volts) {
        intakeDrive.setControl(intakeDrive_Request.withOutput(volts));
    }

    public Command stopIntakeCmd() {
        isRunning = false;
        return new InstantCommand(() -> this.setOutput(0.0), this);
    }

    public Command runIntakeFastCmd() {
        isRunning = true;
        return new InstantCommand(() -> this.setOutput(8), this);
    }

    public Command runIntakeEjectCmd() {
        isRunning = true;
        return new InstantCommand(() -> this.setOutput(-6), this);
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.setSmartDashboardType("Intake");
        sendableBuilder.addDoubleProperty("CurrentSpeed", () -> this.intakeDrive_Request.Output, null);
    }

}
