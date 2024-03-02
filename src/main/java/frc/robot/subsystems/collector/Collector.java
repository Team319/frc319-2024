// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {

  private final CollectorIO io;
  private final CollectorIOInputsAutoLogged inputs = new CollectorIOInputsAutoLogged();

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
    io.updateInputs(inputs);
  }

  public void stop() {
    io.stop();
  }

  public void setCollectorPO(double collectorPO) {
    io.setCollectorPO(collectorPO);
  }

  public void setTunnelPO(double PO) {
    io.setTunnelPO(PO);
  }

   /*  public Command intakeCommand() {
    return ;
  }*/

   /*  public Command ejectCommand() {
    return ;
  }*/


}
