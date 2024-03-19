package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog 
    public static class ShooterIOInputs {
        public double velocityRadPerSec = 0.0;
        public double leftShooterAppliedVolts = 0.0;
        public double rightShooterAppliedVolts = 0.0;

        public double feedRollerAppliedVolts = 0.0;

        public double leftFlywheelAmps = 0.0;
        public double rightFlywheelAmps = 0.0;
        public double feedRollerAmps = 0.0;

        public double leftFlywheelVelocityRadPerSec = 0.0;
        public double rightFlywheelVelocityRadPerSec = 0.0;

        public double leftFlywheelVelocitySetpointRadPerSec = 0.0;
        public double rightFlywheelVelocitySetpointRadPerSec = 0.0;

        public double leftFlywheelVelocityRotationsPerSec = 0.0;
        public double rightFlywheelVelocityRotationsPerSec = 0.0;

        public double leftFlywheelVelocitySetpointRotationsPerSec = 0.0;
        public double rightFlywheelVelocitySetpointRotationsPerSec = 0.0;

        public double wristPosition = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVoltages(double leftFlywheelVolts, double rightFlywheelVolts, double feedVolts) {}
    
    public default void setLeftShooterVelocity(double velocityRadPerSec, double ffVolts) {}

    public default void setRightShooterVelocity(double velocityRadPerSec, double ffVolts) {}

    public default void setShooterVelocity(double shooterVelocity, double ffVolts) {}

    public default double getLeftShooterVelocityRPM(){return 0.0;}

    public default double getRightShooterVelocityRPM(){return 0.0;}

    public default void setFeedVelocity(double velocityRadPerSec, double ffVolts) {}

    public default void stop() {}

    public default void configureFlywheelPID(double kP, double kI, double kD) {}

    public default void configureWristPID(double kP, double kI, double kD, double kFF) {}

    public default void setShooterPO(double PO){}

    public default void setFeedPO(double PO) {}

    public default double getPosition() {return 0.0;}

    public default double getWristPosition() {return 0.0;}

    public default double getCurrentWristSetpoint() {return 0.0;}

    public default void setWristPosition(double position) {}

    public default void setWristPO(double PO) {}

    public default boolean isBeamBreakTripped() { return false;}

    public default double getWristSetpointForDistance(double distance) {return 0.0;};

}
