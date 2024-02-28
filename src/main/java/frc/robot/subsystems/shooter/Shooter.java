// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

//import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase {
  private static final double leftShooterVelocity = 0.0;
  private static ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  private double shooterVelocity;

    
  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) { 
    Shooter.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
      case PROTO:
      case PROTO2:
        ffModel = new SimpleMotorFeedforward(0.33329, 0.00083);
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
    
// double leftShooterVelocity = SmartDashboard.getNumber("leftShooter velocity", 0);
    double rightShooterVelocity = SmartDashboard.getNumber("rightShooter velocity", 0.0);
    //double feedforward = SmartDashboard.getNumber("feedforward volts", 0.0);

    double shooterP = SmartDashboard.getNumber("shooter P", 0.0);
    double shooterI = SmartDashboard.getNumber("shooter I", 0.0);
    double shooterD = SmartDashboard.getNumber("shooter D", 0.0);
    

    double wristPosition = SmartDashboard.getNumber("Wrist Position", 0.0 );

    System.out.println(wristPosition + getPosition());
    
    shooterVelocity = leftShooterVelocity;

    configurePID(shooterP, shooterI, shooterD);
    runShooterVelocity(shooterVelocity);
    //runFeedVelocity(shooterVelocity);

    setFeedVoltage(feedVolts);

    // Update advantageKit logging IO
    //io.updateInputs(inputs);
    //Logger.processInputs("Shooter", inputs);

    //Log shooter speed in RPM
    //Logger.recordOutput("LeftShooterSpeedRPM", getVelocityRPM());
  }

    /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Stops the flywheel. */
  public void setVoltages(double leftShooterVolts ,double rightShooterVolts, double feedVolts) {
    io.setVoltages(leftShooterVolts,rightShooterVolts,feedVolts);
  }

  public void setFeedVoltage(double voltage){
    io.setFeedVoltage(voltage);
    System.err.println("Feed Voltage: " + voltage);
  }

  /** Run closed loop at the specified velocity. */
  public void runShooterVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setShooterVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("ShooterSetpointRPM", velocityRPM);
  }

  
  /** Run closed loop at the specified velocity. */
  public void runFeedVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setFeedVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("FeedSetpointRPM", velocityRPM);
  }

  public void runFeedAtShooterVelocity(){
    runFeedVelocity(this.shooterVelocity);
  }


  public void configurePID(double kP, double kI, double kD) {
    io.configurePID(kP, kI, kD);
  }


  public void setWristPO(double PO){
    io.setWristPO(PO);
  }


//  public static void setFeedPO(double PO){
 //   io.setFeedPO(PO);
 // }

  public double getPosition() {
    return io.getPosition();
  }
  /** Returns the current velocity in RPM. */
 // public double getVelocityRPM() {
 //   return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
 // }


}
