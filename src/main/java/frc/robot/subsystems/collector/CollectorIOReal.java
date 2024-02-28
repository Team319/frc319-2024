// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class CollectorIOReal implements CollectorIO {
  
  private final CANSparkMax collectorLead = new CANSparkMax(20, MotorType.kBrushless);
  private final CANSparkMax collectorFollow = new CANSparkMax(21, MotorType.kBrushless);

  private int currentLimitAmps = 30;
  private double rollerPercentOutputSetpoint = 0.0;
  private boolean isLeadMotorInverted = false;

  public CollectorIOReal() {
    setCollectorFollow();
    setCollectorInversions();
    setup();
  }

  @Override
  public void updateInputs(CollectorIOInputs inputs) {
    inputs.maxCurrentAmps = this.currentLimitAmps;
    inputs.outputCurrentAmps = getCollectorCurrent();
    inputs.appliedPercentOutput = this.rollerPercentOutputSetpoint;
    inputs.isLeadMotorInverted = this.isLeadMotorInverted;
  }

  @Override
  public void setCollectorPO(double PO) {
    this.rollerPercentOutputSetpoint = PO;
    collectorLead.set(this.rollerPercentOutputSetpoint);
  }
  
  public void setCollectorInversions() {
    collectorLead.setInverted(isLeadMotorInverted);
   
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
    
    collectorLead.setSmartCurrentLimit(currentLimitAmps);
  }
}

 