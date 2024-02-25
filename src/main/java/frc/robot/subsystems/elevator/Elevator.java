// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
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
    double elevatorPosition = SmartDashboard.getNumber("Elevator Position", 0.0); //put or get?

    double elevatorP = SmartDashboard.getNumber("Elevator P", 0.0);
    double elevatorI = SmartDashboard.getNumber("Elevator I", 0.0);
    double elevatorD = SmartDashboard.getNumber("Elevator D", 0.0);

  }

  public void stop() {
    io.stop();
  }

  public void setPO(double PO) {
    io.setPO(PO);
  }
}
