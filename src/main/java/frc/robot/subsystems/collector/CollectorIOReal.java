// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorIOReal implements CollectorIO {
  
  private final CANSparkMax collectorLeftLead = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax collectorRightLead = new CANSparkMax(13, MotorType.kBrushless);
  private final CANSparkMax collectorLeftFollow = new CANSparkMax(14, MotorType.kBrushless);
  private final CANSparkMax collectorRightFollow = new CANSparkMax(15, MotorType.kBrushless);
  
  private final CANSparkMax tunnelLead = new CANSparkMax(21, MotorType.kBrushless);
  private final CANSparkMax tunnelFollow = new CANSparkMax(22, MotorType.kBrushless);


  public CollectorIOReal() {
    setCollectorFollow();
    setCollectorInversions();
  }

  @Override
  public void setCollectorPO(double PO) {
    collectorLeftLead.set(PO);
    collectorRightLead.set(PO);
  }
  
  public void setCollectorInversions() {
    collectorLeftLead.setInverted(true);
    collectorLeftFollow.setInverted(true);
   
  }

  public void setCollectorFollow() {
    collectorLeftFollow.follow(collectorLeftLead);
    collectorRightFollow.follow(collectorRightLead);
  }
}

 