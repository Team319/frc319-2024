// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterIOReal implements ShooterIO{

    private final TalonFX shooterLeft = new TalonFX(31); 
    private final TalonFX shooterRight = new TalonFX(32);

    private final CANSparkMax feed = new CANSparkMax(0, MotorType.kBrushless);
    private final SparkPIDController feedPid = feed.getPIDController();



public ShooterIOReal(){

    shooterLeft.setInverted(false);
    shooterRight.setInverted(true);

    feed.setInverted(true);
    
}

@Override
    public void updateInputs(ShooterIOInputs inputs) {}

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
}
