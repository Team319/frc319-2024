package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {

        public double velocityRadPerSec = 0.0;
        public double leftShooterAppliedVolts = 0.0;
        public double rightShooterAppliedVolts = 0.0;

        public double leftFeedAppliedVolts = 0.0;
        public double rightFeedAppliedVolts = 0.0;

        public double[] currentAmps = new double[] {};
        
        public double leftFlywheelVelocityRadPerSec;
        public double rightFlywheelVelocityRadPerSec;
        public double[] leftFlywheelCurrentAmps;
        public double[] rightFlywheelCurrentAmps;
        public double[] leftFeedCurrentAmps;
        public double[] rightFeedCurrentAmps;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVoltages(double leftFlywheelVolts, double rightFlywheelVolts,
                                    double leftFeedVolts, double rightFeedVolts) {}
    
    public default void setLeftShooterVoltage(double velocityRadPerSec, double ffVolts) {}

    public default void setRightShooterVoltage(double velocityRadPerSec, double ffVolts) {}

    public default void setLeftFeedVoltage(double velocityRadPerSec, double ffVolts) {}

    public default void setRightFeedVoltage(double velocityRadPerSec, double ffVolts) {}

    public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

    public default void stop() {}

    public default void configurePID(double kP, double kI, double kD) {}
}
