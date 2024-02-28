
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

//import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorIOSim implements ElevatorIO {

  public static class ElevatorSetpoint {
      public static final double TOP = 0.0;
      public static final double TRAP = 0.0;
      public static final double AMP = 0.0;
      public static final double CLIMB = 0.0;
      public static final double BOTTOM = 0.0;
  }

  public static class ElevatorPIDGains {
    public final double kPUp = 0.0;
    public final double kIUp = 0.0;
    public final double kDUp = 0.0;
    public final double kFFUp = 0.0;

    public final double kPDown = kPUp;
    public final double kIDown = kIUp;
    public final double kDDown = kDUp;
    public final double kFFDown = kFFUp;

  }
    private DCMotorSim elevatorLead = new DCMotorSim(DCMotor.getNEO(1), 1.5, 0.004);
    private DCMotorSim elevatorFollow = new DCMotorSim(DCMotor.getNEO(1), 1.5, 0.004);

  private final ElevatorPIDGains elevatorPIDGains = new ElevatorPIDGains();
  private double positionTargetSetpoint;

  public ElevatorIOSim() {
    setup();
    setFollow();
  }

  public void setup() {

  }

  public void setFollow() {
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs){
    inputs.kPUp = this.elevatorPIDGains.kPUp;
    inputs.kIUp = this.elevatorPIDGains.kIUp;
    inputs.kDUp = this.elevatorPIDGains.kDUp;
    inputs.kFFUp = this.elevatorPIDGains.kFFUp;
    
    inputs.kPDown = this.elevatorPIDGains.kPDown;
    inputs.kIDown = this.elevatorPIDGains.kIDown;
    inputs.kDDown = this.elevatorPIDGains.kDDown;
    inputs.kFFDown = this.elevatorPIDGains.kFFDown;

    inputs.targetPosition = this.positionTargetSetpoint;
    inputs.appliedVoltage = 0.0;
    inputs.outputCurrentAmps = getCurrent();
    inputs.position = getPosition();
    inputs.velocity = getVelocity();

  }

  @Override
  public void stop() {
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double kFF) {

  }

  @Override
  public double getPosition() {
    return 0.0;
  }

  @Override
  public void setPosition(double targetPosition) {
    this.positionTargetSetpoint = targetPosition;
    manageMotion(targetPosition);
  }

  private void manageMotion(double targetPosition) {
    double currentPosition = getPosition();
      if (currentPosition > targetPosition) {
        configurePID(this.elevatorPIDGains.kPUp, this.elevatorPIDGains.kIUp, this.elevatorPIDGains.kDUp, this.elevatorPIDGains.kFFUp);
      }
      else {
        configurePID( this.elevatorPIDGains.kPDown, this.elevatorPIDGains.kIDown, this.elevatorPIDGains.kDDown, this.elevatorPIDGains.kFFDown);
      }
  }

  @Override
  public void setPO(double PO) {
    System.out.println("Elevator setPO: " + PO);
    //elevatorLead.set(PO);
  }

  @Override
  public double getVelocity() {
    return 0.0; //elevatorLead.getEncoder().getVelocity();
  }

  @Override
  public double getCurrent() {
    return 0.0; //elevatorLead.getOutputCurrent();
  }

}
