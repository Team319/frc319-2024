// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {

  private final CollectorIO io;

  /** Creates a new Collector. */
  public Collector(CollectorIO io) {
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

  public void setCollectorPO(double PO) {
    io.setCollectorPO(PO);
    System.out.println("Collector PO : " + PO);
  }

  public double getCollectorPosition() {
  // TODO: return a collector motors position, no motor = no method
    return 0;
  }

  public void setCollectorPosition(double position) {
    io.setCollectorPosition(position);
  }

  public void setTunnelPO(double PO) {
    io.setTunnelPO(PO);
  }
}
