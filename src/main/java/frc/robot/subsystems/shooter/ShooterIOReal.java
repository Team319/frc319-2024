// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOReal implements ShooterIO{

    private final TalonFX shooterLeft = new TalonFX(31); 
    private final TalonFX shooterRight = new TalonFX(32);

    private final CANSparkMax feed = new CANSparkMax(0, MotorType.kBrushless);
    private final SparkPIDController feedPid = feed.getPIDController();

    private final CANSparkMax wrist = new CANSparkMax(33, MotorType.kBrushless);
    private final SparkPIDController wristPid = wrist.getPIDController();
    public RelativeEncoder wristEncoder = wrist.getEncoder();

    public static class WristConstants {
        public static class PID {
          public static final double kP = 0.096155;
          public static final double kI = 0;
          public static final double kD = 0;
  
          public static final int iZone = 0;
          public static final double fGain = 0.00019231;
        }
  
        public static class SetPoints {
          public static final float home = (float)0.0;
          public static final float top = (float)0.0;
          public static final float bottom = (float)0.0;
        }
  
        public static class SoftLimits {
          public static final float forwardSoftLimit = SetPoints.top;
          public static final float reverseSoftLimit = SetPoints.bottom;
        }
  
        public static class Currents {
          public static final int currentMax = 20;
          public static final int currentThreshold = 0;
        }
      }


public ShooterIOReal(){
    setupShooter();
    setupWrist();
    setWristPid();
}

@Override
    public void updateInputs(ShooterIOInputs inputs) {}

    private void setupShooter(){
        shooterLeft.setInverted(false);
        shooterRight.setInverted(true);
        feed.setInverted(true);
    }

    @Override
    public void setFeedVoltage(double ffVolts) {
        feed.setVoltage(ffVolts);
        updateRPM();
        System.err.println("ff voltage: " + ffVolts);
    }
    
    @Override
    public void setVoltages(double leftShooterVolts, double rightShooterVolts, double feedVolts) {
        shooterLeft.setVoltage(leftShooterVolts);
        shooterRight.setVoltage(rightShooterVolts);
        setFeedVoltage(feedVolts);
        updateRPM();
    }

    @Override
    public void setShooterVelocity(double velocityRadPerSec, double ffVolts) {
        shooterLeft.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, ffVolts, 0, false, false, false));
        shooterRight.setControl(
            new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec), 0.0, true, ffVolts, 0, false, false, false));
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
        feed.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) { 
        var shooterConfig = new Slot0Configs();
        shooterConfig.kP = kP;
        shooterConfig.kI = kI;
        shooterConfig.kD = kD;
        
        shooterLeft.getConfigurator().apply(shooterConfig);
        shooterRight.getConfigurator().apply(shooterConfig);
    }

    public void configureFeedPID(double kP, double kI, double kD) {
        feedPid.setP(kP, 0);
        feedPid.setI(kI, 0);
        feedPid.setD(kD, 0);
        feedPid.setFF(0, 0);
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

        wrist.setInverted(false);
        
        wristPid.setFeedbackDevice(wristEncoder);
        wristPid.setOutputRange(-1.0, 1.0);
    }

    private void setWristPid() {
        wristPid.setIZone(WristConstants.PID.iZone);
        wristPid.setP(WristConstants.PID.kP);
        wristPid.setI(WristConstants.PID.kI);
        wristPid.setD(WristConstants.PID.kD);
        wristPid.setFF(WristConstants.PID.fGain);
    }

    @AutoLogOutput(key = "Shooter/Wrist/Position")
    public double getWristPosition() {
        return this.wristEncoder.getPosition();
    }

    @AutoLogOutput(key = "Shooter/Wrist/Velocity")
    public double getWristVelocity() {
        return wristEncoder.getVelocity();
      }

    public void setWristPosition(double targetPosition) {
        wristPid.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
    }

    public void setWristPO(double PO) {
        wrist.set(PO);
    }
}
