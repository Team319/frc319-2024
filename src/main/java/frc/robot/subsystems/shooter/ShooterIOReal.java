// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.WristConstants;

public class ShooterIOReal implements ShooterIO {

    private final TalonFX shooterLeft = new TalonFX(31); 
    private final TalonFX shooterRight = new TalonFX(32);

    private final CANSparkMax feed = new CANSparkMax(34, MotorType.kBrushless);
    private final SparkPIDController feedPid = feed.getPIDController();

    private final CANSparkMax wrist = new CANSparkMax(33, MotorType.kBrushless); // when merging use this value
    private final SparkPIDController wristPid = wrist.getPIDController();
    private final RelativeEncoder wristEncoder = wrist.getEncoder();
    private final DigitalInput beamBreak = new DigitalInput(0);

  public ShooterIOReal() {
      setupShooter();
      setupWrist();
      configureWristPID(WristConstants.PID.kP,WristConstants.PID.kI,WristConstants.PID.kD,WristConstants.PID.kFF );
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
      inputs.leftFlywheelAmps = shooterLeft.getTorqueCurrent().getValueAsDouble();
      inputs.rightFlywheelAmps = shooterRight.getTorqueCurrent().getValueAsDouble();
      inputs.leftFlywheelVelocityRadPerSec = shooterLeft.getVelocity().getValueAsDouble();
      inputs.rightFlywheelVelocityRadPerSec = shooterRight.getVelocity().getValueAsDouble();
      inputs.leftFlywheelVelocitySetpointRadPerSec = shooterLeft.getClosedLoopReference().getValueAsDouble();
      inputs.rightFlywheelVelocitySetpointRadPerSec = shooterRight.getClosedLoopReference().getValueAsDouble();
      inputs.wristPosition = wristEncoder.getPosition();
    }

    private void setupShooter() {
        shooterLeft.setInverted(true);
        shooterRight.setInverted(false);
        feed.setInverted(false);
    }

    
    @Override
    public void setVoltages(double leftShooterVolts, double rightShooterVolts, double feedVolts) {
        shooterLeft.setVoltage(leftShooterVolts);
        shooterRight.setVoltage(rightShooterVolts);
        feed.setVoltage(feedVolts);
        updateRPM();
    }

    @Override
    public void setShooterVelocity(double velocityRadPerSec, double ffVolts) {
        shooterLeft.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, 0, 0, false, false, false));
        shooterRight.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, 0, 0, false, false, false));
        //feed.setVoltage(ffVolts);
        updateRPM();
    }

    public void setFeedVelocity(double velocityRadPerSec, double ffVolts) {
      feedPid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * 60,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
    } 

    @Override
    public void stop() {
        shooterLeft.stopMotor();
        shooterRight.stopMotor();
        wrist.stopMotor();
    }

    @Override
    public void configureFlywheelPID(double kP, double kI, double kD) { 
        var shooterConfig = new Slot0Configs();
        shooterConfig.kP = kP;
        shooterConfig.kI = kI;
        shooterConfig.kD = kD;
        
        shooterLeft.getConfigurator().apply(shooterConfig);
        shooterRight.getConfigurator().apply(shooterConfig);
    }

    public void updateRPM(){
        // rotations per second -> rotations per minute
        SmartDashboard.putNumber("leftShooter rpm", shooterLeft.getVelocity().getValueAsDouble()*60);
        SmartDashboard.putNumber("rightShooter rpm", shooterLeft.getVelocity().getValueAsDouble()*60);
        }

    public void setupWrist(){
        wrist.restoreFactoryDefaults();
        wrist.clearFaults();

        wrist.enableSoftLimit(SoftLimitDirection.kForward, true);
        wrist.enableSoftLimit(SoftLimitDirection.kReverse, true);

        wrist.setSoftLimit(SoftLimitDirection.kForward, WristConstants.SoftLimits.forwardSoftLimit);
        wrist.setSoftLimit(SoftLimitDirection.kReverse, WristConstants.SoftLimits.reverseSoftLimit);

        wrist.setInverted(true);
        
        wristPid.setFeedbackDevice(wristEncoder);
        wristPid.setOutputRange(-1.0, 1.0);
    }

    @Override
    public void configureWristPID(double kP, double kI, double kD, double kFF) {
        wristPid.setIZone(WristConstants.PID.iZone);
        wristPid.setP(kP);
        wristPid.setI(kI);
        wristPid.setD(kD);
        wristPid.setFF(kFF);
    }
    
    @Override
    public void getWristPosition() {
        var wristPosition = wristEncoder.getPosition();
     //System.out.println("Wrist Position" + wristPosition);
    }

    public double getWristVelocity() {
        return wristEncoder.getVelocity();
      }

    public void setWristPosition(double targetPosition) {
        wristPid.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setWristPO(double PO) {
        wrist.set(PO);
    }

    @Override
    public boolean isBeamBreakTripped(){
        return beamBreak.get();
    }

    @Override
    public void setFeedPO(double PO) {
      feed.set(PO);
    }

    public void configureFeedPID(double kP, double kI, double kD) {
      feedPid.setP(kP, 0);
      feedPid.setI(kI, 0);
      feedPid.setD(kD, 0);
      feedPid.setFF(0, 0);
    }
}
