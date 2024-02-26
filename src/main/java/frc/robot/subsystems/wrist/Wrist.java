// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  /** Creates a new Wrist. */
  public Wrist(WristIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
      case PROTO:
      case PROTO2:
      case SIM:
      default:
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    io.stop();
  }

  public void configurePID(double kP, double kI, double kD) {
    io.configurePID(kP, kI, kD);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public void setPO(double PO) {
    io.setPO(PO);
  }

  public void getPosition() {}

  
}
