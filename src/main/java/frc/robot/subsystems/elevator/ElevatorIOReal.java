
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIOReal implements ElevatorIO {
  /** Creates a new ElevatorIOReal. */

  private final CANSparkMax elevatorLead = new CANSparkMax(16, MotorType.kBrushless);
  private final CANSparkMax elevatorFollow = new CANSparkMax(17, MotorType.kBrushless);
  public ElevatorIOReal() {}

  @Override
  public void stop() {}

  /*public void periodic() {
    // This method will be called once per scheduler run
  }*/


}
