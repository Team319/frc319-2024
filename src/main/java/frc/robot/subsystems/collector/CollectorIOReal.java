// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorIOReal implements CollectorIO {
  
  private final CANSparkMax collectorLead = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax collectorFollow = new CANSparkMax(13, MotorType.kBrushless);

  public static final int maxCurrent = 20;

  //private final SparkPIDController collectorPIDController = collectorLead.getPIDController();

  public CollectorIOReal() {
    setCollectorFollow();
    setCollectorInversions();
    setup();
  }

  @Override
  public void setCollectorPO(double PO) {
    collectorLead.set(PO);
  }
  
  public void setCollectorInversions() {
    collectorLead.setInverted(false);
   
  }

  public void setCollectorFollow() {
    collectorFollow.follow(collectorLead);
  }

  public double getCollectorCurrent() {
    return collectorLead.getOutputCurrent();
  }

  public void setup() {
    collectorLead.restoreFactoryDefaults();
    collectorLead.clearFaults();

    //collectorLead.setInverted(true);

    //pidController.setFeedbackDevice(collectorEncoder);
    /*collectorPIDController.setFF(kFF);
    collectorPIDController.setP(kP);
    collectorPIDController.setOutputRange(-1, 1);*/

    collectorLead.setClosedLoopRampRate(0.125);
    collectorLead.setOpenLoopRampRate(0.125);
    
    collectorLead.setSmartCurrentLimit(maxCurrent);
  }
}

 