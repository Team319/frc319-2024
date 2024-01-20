// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

    
  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) { 
    this.io = io;

   
    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        break;  
    
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    double leftShooterVolts = SmartDashboard.getNumber("leftShooter volts", 0.0);
    double rightShooterVolts = SmartDashboard.getNumber("rightShooter volts", 0.0);
    double feedVolts = SmartDashboard.getNumber("feed volts", 0.0);
    
    double leftShooterVelocity = SmartDashboard.getNumber("leftShooter velocity", 0.0);
    double rightShooterVelocity = SmartDashboard.getNumber("rightShooter velocity", 0.0);
    double feedforward = SmartDashboard.getNumber("feedforward volts", 0.0);

    double shooterP = SmartDashboard.getNumber("shooter P", 0.0);
    double shooterI = SmartDashboard.getNumber("shooter I", 0.0);
    double shooterD = SmartDashboard.getNumber("shooter D", 0.0);


   // io.setVoltages(leftShooterVolts, rightShooterVolts, feedVolts);
    io.configurePID(shooterP, shooterI, shooterD);
    double shooterVelocity = leftShooterVelocity;
    //io.setLeftShooterVelocity(leftShooterVelocity, feedforward);
    //io.setRightShooterVelocity(rightShooterVelocity, feedforward);
    io.setLeftShooterVelocity(shooterVelocity, feedforward);
    io.setRightShooterVelocity(shooterVelocity, feedforward);

    io.setFeedVoltage(feedVolts);


    ///io.updateInputs(inputs);
    //Logger.processInputs("Shooter", inputs);

    //Log flywheel speed in RPM
    //Logger.recordOutput("LeftShooterSpeedRPM", getVelocityRPM());
  }

    /** Stops the flywheel. */
  public void stop() {
    //io.stop();
  }

  /** Stops the flywheel. */
  public void setVoltages(double leftShooterVolts ,double rightShooterVolts, double feedVolts) {
    io.setVoltages(leftShooterVolts,rightShooterVolts,feedVolts);

  }

  public void setFeedVoltage(double voltage){
    io.setFeedVoltage(voltage);
  }

  public void setPercentOutput(double rightShooterPO, double leftShooterPO, double feedPO) {

  }


  /** Returns the current velocity in RPM. */
  //public double getVelocityRPM() {
    //return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  //}
  
}
