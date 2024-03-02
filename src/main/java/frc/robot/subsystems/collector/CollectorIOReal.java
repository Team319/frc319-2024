// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class CollectorIOReal implements CollectorIO {
  
  private final CANSparkMax collectorLead = new CANSparkMax(20, MotorType.kBrushless);
  private final CANSparkMax collectorFollow = new CANSparkMax(21, MotorType.kBrushless);

  private final CANSparkMax feed = new CANSparkMax(34, MotorType.kBrushless);
  private final SparkPIDController feedPid = feed.getPIDController();


  public DigitalInput beamBreak = new DigitalInput(1);


  public LinearFilter currentFilter = LinearFilter.movingAverage(10);


  private int currentLimitAmps = 30;
  private double rollerPercentOutputSetpoint = 0.0;
  private double rollerPercentOutputSetpoint2 = 0.0;
  private boolean isLeadMotorInverted = false;



  public CollectorIOReal() {
    setCollectorFollow();
    setCollectorInversions();
    setup();
  }

  @Override
  public void updateInputs(CollectorIOInputs inputs) {
    inputs.feedRollerAmps = feed.getOutputCurrent();
    inputs.maxCurrentAmps = this.currentLimitAmps;
    inputs.outputCurrentAmps = getCollectorCurrent();
    inputs.appliedPercentOutput = this.rollerPercentOutputSetpoint;
    inputs.isLeadMotorInverted = this.isLeadMotorInverted;

  }

  @Override
  public void setCollectorPO(double collectorPO, double feedPO) {
    this.rollerPercentOutputSetpoint = collectorPO;
    collectorLead.set(this.rollerPercentOutputSetpoint);

    this.rollerPercentOutputSetpoint2 = feedPO;
    feed.set(this.rollerPercentOutputSetpoint2);

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
    feed.setInverted(false);
  }

  // Feed motor stuff
  @Override
  public void setFeedVoltage(double ffVolts) {
    feed.setVoltage(ffVolts);
  }

  public void configureFeedPID(double kP, double kI, double kD) {
    feedPid.setP(kP, 0);
    feedPid.setI(kI, 0);
    feedPid.setD(kD, 0);
    feedPid.setFF(0, 0);
  }


}

 