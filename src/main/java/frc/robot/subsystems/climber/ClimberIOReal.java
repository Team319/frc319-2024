// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ClimberConstants;

public class ClimberIOReal implements ClimberIO {
  /** Creates a new CollectorIOReal. */

  private final CANSparkMax leftClimber = new CANSparkMax(62, MotorType.kBrushless);
  private final CANSparkMax rightClimber = new CANSparkMax(61, MotorType.kBrushless);

  private final RelativeEncoder climberEncoder = rightClimber.getEncoder();
  private final SparkPIDController climberPIDController = rightClimber.getPIDController();

  private double positionTargetSetpoint;

  public ClimberIOReal() {
    setup();
  }

  // Start methods here
    public void setup() {

    leftClimber.follow(rightClimber, true);

    rightClimber.restoreFactoryDefaults();
    rightClimber.clearFaults();

    rightClimber.setInverted(false);

    rightClimber.setSmartCurrentLimit(30);

    rightClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);

    climberPIDController.setFeedbackDevice(climberEncoder);

    rightClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);

    rightClimber.setSoftLimit(SoftLimitDirection.kForward, (float)ClimberConstants.Setpoints.top);
    rightClimber.setSoftLimit(SoftLimitDirection.kReverse, (float)ClimberConstants.Setpoints.bottom);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.kPUp = ClimberConstants.PID.kPUp;
    inputs.kIUp = ClimberConstants.PID.kIUp;
    inputs.kDUp = ClimberConstants.PID.kDUp;
    inputs.kFFUp = ClimberConstants.PID.kFFUp;
    
    inputs.kPDown = ClimberConstants.PID.kPDown;
    inputs.kIDown = ClimberConstants.PID.kIDown;
    inputs.kDDown = ClimberConstants.PID.kDDown;
    inputs.kFFDown = ClimberConstants.PID.kFFDown;

    inputs.targetPosition = this.positionTargetSetpoint;
    inputs.appliedVoltage = rightClimber.getAppliedOutput();
    inputs.outputCurrentAmps = getCurrent();
    inputs.position = getPosition();
    inputs.velocity = getVelocity();

  }

  @Override
  public void stop() {
    rightClimber.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double kFF) {
    climberPIDController.setP(kP);
    climberPIDController.setI(kI);
    climberPIDController.setI(kD);
    climberPIDController.setFF(kFF);
  }

  @Override
  public double getPosition() {
    return this.climberEncoder.getPosition();
  }

  @Override
  public void setPosition(double targetPosition) {
    this.positionTargetSetpoint = targetPosition;
    manageMotion(targetPosition);
    climberPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void setPO(double PO) {
    rightClimber.set(PO);
  }

  @Override
  public double getVelocity() {
    return rightClimber.getEncoder().getVelocity();
  }

  @Override
  public double getCurrent() {
    return rightClimber.getOutputCurrent();
  }

  private void manageMotion(double targetPosition) {
    double currentPosition = getPosition();
      if (currentPosition > targetPosition) {
        configurePID(ClimberConstants.PID.kPUp, ClimberConstants.PID.kIUp, ClimberConstants.PID.kDUp, ClimberConstants.PID.kFFUp);
      }
      else {
        configurePID( ClimberConstants.PID.kPDown, ClimberConstants.PID.kIDown, ClimberConstants.PID.kDDown, ClimberConstants.PID.kFFDown);
      }
  }
}
