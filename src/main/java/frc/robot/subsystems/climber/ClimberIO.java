// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

    @AutoLog
    public static class ClimberIOInputs {
        public double kPUp = 0.0;  // Power applied to motor
        public double kIUp = 0.0;  // margin of error in motor
        public double kDUp = 0.0;  // Makes the graph line smooth from point A to point B
        public double kFFUp = 0.0; // Feedforward value

        public double kPDown = 0.0;  // Power applied to motor
        public double kIDown = 0.0;  // margin of error in motor
        public double kDDown = 0.0;  // Makes the graph line smooth from point A to point B
        public double kFFDown = 0.0; // Feedforward value

        public double targetPosition = 0.0; // Target position of the elevator
        public double appliedVoltage = 0.0; // Voltage applied to the motor
        public double outputCurrentAmps = 0.0; // Current applied to the motor
        public double position = 0.0; // Position of the elevator
        public double velocity = 0.0; // Velocity of the elevator

    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void stop() {}

    public default void configurePID(double kP, double kI, double kD, double kFF) {}

    public default void setPosition(double targetPosition) {}

    public default double getPosition() {return 0.0;}

    public default double getVelocity() {return 0.0;}

    public default double getCurrent() {return 0.0;}

    public default void setVoltage(double voltage) {}

    public default void setLeftPO(double leftPO) {}

    public default void setRightPO (double rightPO) {}

    
}