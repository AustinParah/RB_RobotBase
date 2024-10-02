package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements Lifecycle, Sendable {
    private final ShooterHelper upper = new ShooterHelper("ShooterSubsystem", "Upper", new TalonFX(50));
    private final ShooterHelper lower = new ShooterHelper("ShooterSubsystem", "Lower", new TalonFX(55));

}
