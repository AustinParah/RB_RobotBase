package frc.robot.subsystems.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PIDHelper {
    private final String name;
    private final boolean useDashboard;

    public double kF = 0.0;
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;

    public double kV = 0.0;
    public double kA = 0.0;

    public PIDHelper(String name) {
        this(name, true);
    }

    public PIDHelper(String name, boolean useDashboard) {
        this.name = name;
        this.useDashboard = useDashboard;
    }

    public void initialize(double kP, double kI, double kD, double kF, double kV, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kV = kV;
        this.kA = kA;

        if (this.useDashboard) {
            SmartDashboard.setDefaultNumber(name + "/kP", kP);
            SmartDashboard.setDefaultNumber(name + "/kI", kI);
            SmartDashboard.setDefaultNumber(name + "/kD", kD);
            SmartDashboard.setDefaultNumber(name + "/kF", kF);
            SmartDashboard.setDefaultNumber(name + "/kV", kV);
            SmartDashboard.setDefaultNumber(name + "/kA", kA);
        }
    }

    public boolean updateValuesFromDashboard() {
        if (!this.useDashboard) {
            // Do nothing if we don't use dashboard
            return false;
        }
        double p = SmartDashboard.getNumber(name + "/kP", kP);
        double i = SmartDashboard.getNumber(name + "/kI", kI);
        double d = SmartDashboard.getNumber(name + "/kD", kD);
        double ff = SmartDashboard.getNumber(name + "/kF", kF);
        double v = SmartDashboard.getNumber(name + "/kV", kV);
        double a = SmartDashboard.getNumber(name + "/kA", kA);

        DoubleChanger changer = new DoubleChanger();
        this.kP = changer.change(this.kP, p);
        this.kI = changer.change(this.kI, i);
        this.kD = changer.change(this.kD, d);
        this.kF = changer.change(this.kF, ff);
        this.kV = changer.change(this.kV, v);
        this.kA = changer.change(this.kA, a);
        return changer.isChanged();
    }


    public void updatePIDController(PIDController pid) {
        pid.setPID(this.kP, this.kI, this.kD);
    }

    public void updateProfiledPIDController(ProfiledPIDController pid) {
        pid.setPID(this.kP, this.kI, this.kD);
    }

    public void overrideP(double value) {
        SmartDashboard.putNumber(name + "/kP", value);
        this.kP = value;
    }

    public Slot0Configs updateConfiguration(Slot0Configs config) {
        config
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKV(kV)
            .withKA(kA);
        return config;
    }

}
