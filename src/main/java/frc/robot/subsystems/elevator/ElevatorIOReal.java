
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

//import org.littletonrobotics.junction.AutoLogOutput;

//import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

  public static class ElevatorSetpoint {
      public static final double TOP = 90.0; // 130.0
      public static final double TRAP = TOP;
      public static final double AMP = 47.85;
      public static final double CLIMB = 5.0;
      public static final double BOTTOM = 2.0; // 2.0
  }

  public static class ElevatorPIDGains {
    public final double kPUp = 0.2;
    public final double kIUp = 0.0;
    public final double kDUp = 0.0;
    public final double kFFUp = 0.0;

    public final double kPDown = kPUp;
    public final double kIDown = kIUp;
    public final double kDDown = kDUp;
    public final double kFFDown = kFFUp;

  }

  private final CANSparkMax elevatorLead = new CANSparkMax(40, MotorType.kBrushless);
  private final CANSparkMax elevatorFollow = new CANSparkMax(41, MotorType.kBrushless);

  private final RelativeEncoder elevatorEncoder = elevatorLead.getEncoder();
  private final SparkPIDController elevatorPIDController = elevatorLead.getPIDController();

  private double positionTargetSetpoint;

  public ElevatorIOReal() {
    setup();
    setFollow();
  }

  public void setup() {

    elevatorLead.restoreFactoryDefaults();
    elevatorFollow.restoreFactoryDefaults();

    elevatorLead.clearFaults();
    elevatorFollow.clearFaults();

    elevatorLead.setInverted(false);

    elevatorLead.enableSoftLimit(SoftLimitDirection.kForward, true);
    elevatorLead.enableSoftLimit(SoftLimitDirection.kReverse, true);

    elevatorLead.setSoftLimit(SoftLimitDirection.kForward, (float)ElevatorConstants.Setpoints.top);
    elevatorLead.setSoftLimit(SoftLimitDirection.kReverse, (float)ElevatorConstants.Setpoints.bottom);

    elevatorLead.setSmartCurrentLimit(30);
    elevatorFollow.setSmartCurrentLimit(30);
    elevatorLead.setIdleMode(IdleMode.kBrake);
    elevatorFollow.setIdleMode(IdleMode.kBrake);

    elevatorPIDController.setFeedbackDevice(elevatorEncoder);
    elevatorPIDController.setOutputRange(-1.0, 1.0);

    configurePID(ElevatorConstants.PID.kPUp,ElevatorConstants.PID.kIUp,ElevatorConstants.PID.kDUp,ElevatorConstants.PID.kFFUp);
  }

  public void setFollow() {
    elevatorFollow.follow(elevatorLead, true);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs){
    inputs.kPUp = ElevatorConstants.PID.kPUp;
    inputs.kIUp = ElevatorConstants.PID.kIUp;
    inputs.kDUp = ElevatorConstants.PID.kDUp;
    inputs.kFFUp = ElevatorConstants.PID.kFFUp;
    
    inputs.kPDown = ElevatorConstants.PID.kPDown;
    inputs.kIDown = ElevatorConstants.PID.kIDown;
    inputs.kDDown = ElevatorConstants.PID.kDDown;
    inputs.kFFDown = ElevatorConstants.PID.kFFDown;

    inputs.targetPosition = this.positionTargetSetpoint;
    inputs.appliedVoltage = elevatorLead.getAppliedOutput();
    inputs.outputCurrentAmps = getCurrent();
    inputs.position = getPosition();
    inputs.velocity = getVelocity();

  }

  @Override
  public void stop() {
    elevatorLead.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double kFF) {
    elevatorPIDController.setP(kP);
    elevatorPIDController.setI(kI);
    elevatorPIDController.setI(kD);
    elevatorPIDController.setFF(kFF);
  }

  @Override
  public double getPosition() {
    return this.elevatorEncoder.getPosition();
  }

  @Override
  public void setPosition(double targetPosition) {
    this.positionTargetSetpoint = targetPosition;
    manageMotion(targetPosition);
    elevatorPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

   @Override
  public void setPO(double PO) {
    elevatorLead.set(PO);
  }

  @Override
  public double getVelocity() {
    return elevatorLead.getEncoder().getVelocity();
  }

  @Override
  public double getCurrent() {
    return elevatorLead.getOutputCurrent();
  }

  private void manageMotion(double targetPosition) {
    double currentPosition = getPosition();
      if (currentPosition > targetPosition) {
        configurePID(ElevatorConstants.PID.kPUp, ElevatorConstants.PID.kIUp, ElevatorConstants.PID.kDUp, ElevatorConstants.PID.kFFUp);
      }
      else {
        configurePID( ElevatorConstants.PID.kPDown, ElevatorConstants.PID.kIDown, ElevatorConstants.PID.kDDown, ElevatorConstants.PID.kFFDown);
      }
  }

 

}
