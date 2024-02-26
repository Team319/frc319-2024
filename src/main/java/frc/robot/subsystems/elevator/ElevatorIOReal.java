
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

//import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorIOReal implements ElevatorIO {
  /** Creates a new ElevatorIOReal. */

  private final static CANSparkMax elevatorLead = new CANSparkMax(40, MotorType.kBrushless);
  private final CANSparkMax elevatorFollow = new CANSparkMax(41, MotorType.kBrushless);


  private final static RelativeEncoder elevatorEncoder = elevatorLead.getEncoder();
  private final SparkPIDController elevatorPIDController = elevatorLead.getPIDController();

  public static class ElevatorSetpoint {
    public static final double TOP = 0.0;
    public static final double TRAP = 0.0;
    public static final double AMP = 0.0;
    public static final double CLIMB = 0.0;
    public static final double BOTTOM = 0.0;
  }

  public ElevatorIOReal() {
    setup();
    setFollow();
  }

  @Override
  public void stop() {
    elevatorLead.stopMotor();
  }

  public void upPID() {
    elevatorPIDController.setP(kPUp);
    elevatorPIDController.setI(kIUp);
    elevatorPIDController.setI(kDUp);
    elevatorPIDController.setFF(kFFUp);
  }

  public void downPID() {
    elevatorPIDController.setP(kPDown);
    elevatorPIDController.setI(kIDown);
    elevatorPIDController.setI(kDDown);
    elevatorPIDController.setFF(kFFDown);
  }

  public static double getCurrentPosition() {
    return elevatorEncoder.getPosition();
  }

  public void setPosition(double targetPosition) {
    manageMotion(targetPosition);
    elevatorPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  private void manageMotion(double targetPosition) {
    double currentPosition = getCurrentPosition();
      if (currentPosition > targetPosition) {
        upPID();
      }
      else {
        downPID();
      }
  }

  public void setup() {

    elevatorLead.restoreFactoryDefaults();
    elevatorFollow.restoreFactoryDefaults();

    elevatorLead.clearFaults();
    elevatorFollow.clearFaults();

    elevatorLead.setInverted(false);

   // elevatorLead.enableSoftLimit(SoftLimitDirection.kForward, true);
   // elevatorLead.enableSoftLimit(SoftLimitDirection.kReverse, true);

   // elevatorLead.setSoftLimit(SoftLimitDirection.kForward, (float)ElevatorSetpoint.TOP);
    //elevatorLead.setSoftLimit(SoftLimitDirection.kReverse, (float)ElevatorSetpoint.BOTTOM);

    elevatorLead.setSmartCurrentLimit(30);
    elevatorFollow.setSmartCurrentLimit(30);

    elevatorPIDController.setFeedbackDevice(elevatorEncoder);
    elevatorPIDController.setOutputRange(-1.0, 1.0);
  }

  public void setPO(double PO) {
    elevatorLead.set(PO);
  }

  @AutoLogOutput(key = "Elevator/leadVelocity")
  public double getVelocity() {
    return elevatorLead.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "Elevator/leadAmps")
  public double getElevatorCurrent() {
    return elevatorLead.getOutputCurrent();
  }

  public void setFollow() {
    elevatorFollow.follow(elevatorLead, true);
  }

}
