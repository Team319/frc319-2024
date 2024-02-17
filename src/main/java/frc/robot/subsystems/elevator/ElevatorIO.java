
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {

    public static final double kPUp = 0.0;  // Power applied to motor
    public static final double kIUp = 0.0;  // margin of error in motor
    public static final double kDUp = 0.0;  // Makes the graph line smooth from point A to point B
    public static final double kFFUp = 0.0; // Feedforward value

    public static final double kPDown = 0.0;  // Power applied to motor
    public static final double kIDown = 0.0;  // margin of error in motor
    public static final double kDDown = 0.0;  // Makes the graph line smooth from point A to point B
    public static final double kFFDown = 0.0; // Feedforward value

    public static final double maxVel = 0.0;
    public static final double minVel = 0.0;
    public static final double maxAcc = 0.0;
    public static final double maxErr = 0.0;
    public static final int smartMotionSlot = 0;

    @AutoLog
    public static class ElevatorIOInputs {}

    public default void stop() {}

    public default void configurePID(double kP, double kI, double kD) {}

    public default void setPosition(double targetPosition) {}

    public default void setVoltage(double voltage) {}

    public default void setup() {}

    public default void setInverted() {}

    
}
