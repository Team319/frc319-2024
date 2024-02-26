// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

/** Add your docs here. */
public interface WristIO {

    public default void setup() {}

    public default void configurePID(double kP, double kI, double kD) {}

    public default void stop() {}

    public default void setPosition(double position) {}

    public default void setPO(double PO) {}

}
