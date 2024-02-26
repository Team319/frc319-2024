// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristIOReal implements WristIO {

  private final CANSparkMax wrist = new CANSparkMax(0, MotorType.kBrushless);
  private final RelativeEncoder wristEncoder = wrist.getEncoder();
  private final SparkPIDController wristPIDController = wrist.getPIDController();

  private class WristPID {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }

  /** Creates a new WristIOReal. */
  public WristIOReal() {
    setSoftLimits();
  }

  public double getPosition() {
    return wristEncoder.getPosition();
  }

  @Override
  public void stop() {
    wrist.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    wristPIDController.setP(WristPID.kP);
    wristPIDController.setI(WristPID.kI);
    wristPIDController.setD(WristPID.kD);
  }

  @Override
  public void setPosition(double position) {
    wristEncoder.setPosition(position);
  }

  @Override
  public void setPO(double PO) {
    wrist.set(PO);
  }

  public void setSoftLimits() {
    wrist.setSoftLimit(SoftLimitDirection.kForward, 0);
    wrist.setSoftLimit(SoftLimitDirection.kReverse, 0);
  }
}
