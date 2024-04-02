// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

//import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  private static final double leftShooterVelocity = 0.0;
  private static ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  private double shooterVelocity;

    private static final LoggedTunableNumber wrist_setpoint = new LoggedTunableNumber("Wrist/setpoint", 0.0);
   // private static final LoggedTunableNumber shooter_setpoint = new LoggedTunableNumber("Flywheel/setpoint", 0.0);


  private static final LoggedTunableNumber wrist_kP = new LoggedTunableNumber("Wrist/kP", WristConstants.PID.kP);
  private static final LoggedTunableNumber wrist_kI = new LoggedTunableNumber("Wrist/kI", WristConstants.PID.kI);
  private static final LoggedTunableNumber wrist_kD = new LoggedTunableNumber("Wrist/kD", WristConstants.PID.kD);
  private static final LoggedTunableNumber wrist_kFF = new LoggedTunableNumber("Wrist/kFF", WristConstants.PID.kFF);


//  private static final LoggedTunableNumber flywheel_kP = new LoggedTunableNumber("Flywheel/kP", 0.0);
 // private static final LoggedTunableNumber flywheel_kI = new LoggedTunableNumber("Flywheel/kI", 0.0);
 // private static final LoggedTunableNumber flywheel_kD = new LoggedTunableNumber("Flywheel/kD", 0.0);
    
  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) { 
    Shooter.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
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
    //double leftShooterVolts = SmartDashboard.getNumber("leftShooter volts", 0.0);
    //double rightShooterVolts = SmartDashboard.getNumber("rightShooter volts", 0.0);
    //double feedVolts = SmartDashboard.getNumber("feed volts", 0.0);
    
    //double leftShooterVelocity = SmartDashboard.getNumber("leftShooter velocity", 0);
    //double rightShooterVelocity = SmartDashboard.getNumber("rightShooter velocity", 0.0);
    //double feedforward = SmartDashboard.getNumber("feedforward volts", 0.0);

    shooterVelocity = leftShooterVelocity;

    //LoggedTunableNumber.ifChanged(hashCode(), pid -> configureFlywheelPID(pid[0], pid[1], pid[2]) , flywheel_kP, flywheel_kI, flywheel_kD);

    LoggedTunableNumber.ifChanged(hashCode(), pid -> configureWristPID(pid[0], pid[1], pid[2],pid[3]) , wrist_kP, wrist_kI, wrist_kD, wrist_kFF);

    LoggedTunableNumber.ifChanged(hashCode(), setpoint -> setWristPosition(setpoint[0]) , wrist_setpoint);

    //LoggedTunableNumber.ifChanged(hashCode(), setpoint -> setShooterVelocity(setpoint[0]) , shooter_setpoint);


    //runShooterVelocity(shooterVelocity);
    getWristPosition();

    //runFeedVelocity(shooterVelocity);

    //setFeedVoltage(feedVolts);

    // Update advantageKit logging IO
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    //Log shooter speed in RPM
    Logger.recordOutput("LeftShooterSpeedRPM", getVelocityRPM());
  }

    /** Stops all motors. */
  public void stop() {
    io.stop();
  }

// ====== Flywheel =========

  /** Stops the flywheel. */
  public void setVoltages(double leftShooterVolts ,double rightShooterVolts, double feedVolts) {
    io.setVoltages(leftShooterVolts,rightShooterVolts,feedVolts);
  }

  //public void setFeedVoltage(double voltage){
 //  io.setFeedVoltage(voltage);
 // }

  /** Run closed loop at the specified velocity. */
  public void setShooterVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setShooterVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Shooter/SetpointRPM", velocityRPM);
    Logger.recordOutput("Shooter/SetpointRadPerSecond", velocityRadPerSec );
  }

  
  /** Run closed loop at the specified velocity. */
  public void runFeedVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setFeedVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Shooter/Feed/SetpointRPM", velocityRPM);
  }

  public void runFeedAtShooterVelocity(){
    runFeedVelocity(this.shooterVelocity);
  }

  public void configureFlywheelPID(double kP, double kI, double kD) {
    io.configureFlywheelPID(kP, kI, kD);
  }

  public void configureWristPID(double kP, double kI, double kD, double kFF) {
    io.configureWristPID(kP, kI, kD, kFF);
  }

// ====== Wrist =========

  public void setWristPO(double PO){
    io.setWristPO(PO);
  }

  public void setWristPosition(double position){
    io.setWristPosition(position);
  }

  public double getWristPosition() {
    return io.getWristPosition();
  }

  public double getCurrentWristSetpoint() {
    return io.getCurrentWristSetpoint();
  }

  public void setFeedPO(double PO){
    io.setFeedPO(PO);
  }

 public double getPosition() {
  return io.getPosition();
}
  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return (io.getLeftShooterVelocityRPM());
  }

 public boolean isBeamBreakTripped() {
  return io.isBeamBreakTripped();
 }

 public double getWristSetpointForDistance(double distance) {
  return io.getWristSetpointForDistance(distance);
}

   public Command shootCommand(double rpm) {
    return Commands.runOnce(
      () -> {
        setShooterVelocity(3000); // TODO : Generic number

      },
      this
    );
  }

     public Command feedCommand() {
    return Commands.runOnce(
      () -> {
        runFeedVelocity(2000);
      }
    );
  }
}
