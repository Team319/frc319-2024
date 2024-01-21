package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    

    public static final double kS = 0.0; // Overcomes Static friction
    public static final double kV = 0.0; // A velocity target of 1 rps results in 0.12 V output [Ctre documentation]
    public static final double kA = 0.0; // The request acceleration/acceleration value?
    public static final double kP = 0.0; // Power applied to motor
    public static final double kI = 0.0; // margin of error in motor
    public static final double kD = 0.0; // Nico question? Ask about value later
    
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

    public default void setFeedVoltage( double ffVolts) {}

    public default void setVoltages(double leftFlywheelVolts, double rightFlywheelVolts, double feedVolts) {}
    
    public default void setLeftShooterVelocity(double velocityRadPerSec, double ffVolts) {}

    public default void setRightShooterVelocity(double velocityRadPerSec, double ffVolts) {}

    public default void setShooterVelocity(double velocityRadPerSec, double ffVolts) {}

    public default void setFeedVelocity(double velocityRadPerSec, double ffVolts) {}

    public default void stop() {}

    public default void configurePID(double kP, double kI, double kD) {}
}
