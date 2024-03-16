
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

public class ElevatorIOReal implements ElevatorIO {

  public static class ElevatorSetpoint {
      public static final double TOP = 90.0; // 130.0
      public static final double TRAP = TOP;
      public static final double AMP = 47.785;
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

  private final ElevatorPIDGains elevatorPIDGains = new ElevatorPIDGains();
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

   elevatorLead.setSoftLimit(SoftLimitDirection.kForward, (float)ElevatorSetpoint.TOP);
   elevatorLead.setSoftLimit(SoftLimitDirection.kReverse, (float)ElevatorSetpoint.BOTTOM);

    elevatorLead.setSmartCurrentLimit(30);
    elevatorFollow.setSmartCurrentLimit(30);
    elevatorLead.setIdleMode(IdleMode.kBrake);
    elevatorFollow.setIdleMode(IdleMode.kBrake);

    elevatorPIDController.setFeedbackDevice(elevatorEncoder);
    elevatorPIDController.setOutputRange(-1.0, 1.0);

    configurePID(this.elevatorPIDGains.kPUp,this.elevatorPIDGains.kIUp,this.elevatorPIDGains.kDUp,this.elevatorPIDGains.kFFUp);
  }

  public void setFollow() {
    elevatorFollow.follow(elevatorLead, true);
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





}
