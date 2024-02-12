// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristIOReal implements WristIO {

  private final TalonFX wrist = new TalonFX(23);

  /** Creates a new WristIOReal. */
  public WristIOReal() {}

  @Override
  public void stop() {
    wrist.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {}

  @Override
  public void setPosition(double position) {
    wrist.setPosition(position);
  }

  @Override
  public void setPO(double PO) {}
}
