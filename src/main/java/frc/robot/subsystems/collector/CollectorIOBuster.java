// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotContainer;

public class CollectorIOBuster implements CollectorIO {

  private DigitalInput beamBreakBuster = new DigitalInput(0);

  /** Creates a new CollectorIOBuster. */
  public CollectorIOBuster() {}

  @Override
  public boolean isBeamBreakTripped() {
    return beamBreakBuster.get();
  }
}
