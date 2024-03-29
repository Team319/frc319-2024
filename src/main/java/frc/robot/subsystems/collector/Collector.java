// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
      case SIM:
      default:
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Collector",inputs);
  }

  public void stop() {
    io.stop();
  }

  public void setRollersPO(double collectorPO) {
    io.setRollersPO(collectorPO);
  }

  public void setLowerRollersPO(double collectorPO) {
    io.setLowerPO(collectorPO);
  }

  public void setTunnelRollersPO(double collectorPO) {
    io.setTunnelPO(collectorPO);
  }

  public Command setCollector(double PO) {
    return Commands.runOnce(
      () -> {
        setRollersPO(PO);
      },
      this
      ); 
  }

  public boolean isBeamBreakTripped(){
    return io.isBeamBreakTripped();
  }

}
