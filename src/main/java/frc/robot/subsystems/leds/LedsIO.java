// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

public interface LedsIO {

  public default void stop() {}

  public default void setColor(int rValue, int gValue, int bValue) {}

  public default void allianceIdleColor() {}

}
