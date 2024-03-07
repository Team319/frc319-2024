// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Leds extends SubsystemBase {

  private final LedsIO io;

  /** Creates a new leds. */
  public Leds(LedsIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
      case SIM:
      default:
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //setColor(0xFF, 0xFF, 0x00);
  }

  public void stop() {
    io.stop();
  }

  public void setColor(int rValue, int gValue, int bValue) {
    io.setColor(rValue, gValue, bValue);
  }

  public void allianceIdleColor() {
    io.allianceIdleColor();
  }

}
