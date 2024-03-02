// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class CollectorIOReal implements CollectorIO {
  
  private final CANSparkMax lowerRollerMotor = new CANSparkMax(20, MotorType.kBrushless);
  private final CANSparkMax tunnelRollerMotor = new CANSparkMax(21, MotorType.kBrushless);

  private DigitalInput beamBreak = new DigitalInput(1);

  public LinearFilter currentFilter = LinearFilter.movingAverage(10);

  private int currentLimitAmps = 30;

  private boolean isLeadMotorInverted = false;



  public CollectorIOReal() {
    setCollectorInversions();
    setup();
  }

  @Override
  public void updateInputs(CollectorIOInputs inputs) {
    inputs.maxCurrentAmps = this.currentLimitAmps;
    inputs.outputCurrentAmps = getCollectorCurrent();
    inputs.isLeadMotorInverted = this.isLeadMotorInverted;

  }

  @Override
  public void setRollersPO(double PO) {
    lowerRollerMotor.set(PO);
    tunnelRollerMotor.set(PO);

  }

  @Override
  public void setLowerPO(double PO) {
    lowerRollerMotor.set(PO);
  }

  @Override
  public void setTunnelPO(double PO) {
    tunnelRollerMotor.set(PO);
  }
  
  public void setCollectorInversions() {
    lowerRollerMotor.setInverted(isLeadMotorInverted);
   
  }

  public double getCollectorCurrent() {
    return lowerRollerMotor.getOutputCurrent();
  }

  public void setup() {
    lowerRollerMotor.restoreFactoryDefaults();
    lowerRollerMotor.clearFaults();

    //lowerRollerMotor.setInverted(true);

    //pidController.setFeedbackDevice(collectorEncoder);
    /*collectorPIDController.setFF(kFF);
    collectorPIDController.setP(kP);
    collectorPIDController.setOutputRange(-1, 1);*/

    lowerRollerMotor.setClosedLoopRampRate(0.125);
    lowerRollerMotor.setOpenLoopRampRate(0.125);
    
    lowerRollerMotor.setSmartCurrentLimit(currentLimitAmps);
  }

    @Override
    public boolean isBeamBreakTripped(){
        return beamBreak.get();
    }

}

 