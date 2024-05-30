// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOReal implements ClimberIO {

  public static class ClimberSetpoint {
    public static final double TOP = 29.0; // 
    public static final double BOTTOM = 2.0; // 2.0
}

public static class ClimberPIDGains {
  public final double kPUp = 0.2;
  public final double kIUp = 0.0;
  public final double kDUp = 0.0;
  public final double kFFUp = 0.0;

  public final double kPDown = kPUp;
  public final double kIDown = kIUp;
  public final double kDDown = kDUp;
  public final double kFFDown = kFFUp;
}

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

    rightClimber.restoreFactoryDefaults();
    leftClimber.restoreFactoryDefaults();

    rightClimber.clearFaults(); //Look at refresh rate and lower it if needed to stop error
    leftClimber.clearFaults();

    for (int i = 0; i < 30; i++) {
        rightClimber.setInverted(false); 
        leftClimber.setInverted(true);
    }

    while(rightClimber.getInverted()){ // if the motor is inverted, keep setting it NOT inverted until it is correct
      rightClimber.setInverted(true);
      Timer.delay(0.001);
    }

    while(!leftClimber.getInverted()){ // if the motor is NOT inverted, keep setting it inverted until it is correct
      leftClimber.setInverted(false);
      Timer.delay(0.001);
    }

    rightClimber.setSmartCurrentLimit(30);
    leftClimber.setSmartCurrentLimit(30);

    rightClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);

    climberPIDController.setFeedbackDevice(climberEncoder);
    climberPIDController.setOutputRange(1.0, -1.0);

    rightClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);

    leftClimber.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftClimber.enableSoftLimit(SoftLimitDirection.kReverse, true);

    rightClimber.setSoftLimit(SoftLimitDirection.kForward, (float)ClimberConstants.Setpoints.top);
    rightClimber.setSoftLimit(SoftLimitDirection.kReverse, (float)ClimberConstants.Setpoints.bottom);

    leftClimber.setSoftLimit(SoftLimitDirection.kForward, (float)ClimberConstants.Setpoints.top);
    leftClimber.setSoftLimit(SoftLimitDirection.kReverse, (float)ClimberConstants.Setpoints.bottom);
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
  public void setRightPO(double rightPO) {
    rightClimber.set(rightPO);
  }
  @Override
  public void setLeftPO(double leftPO) {
    leftClimber.set(leftPO);
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

