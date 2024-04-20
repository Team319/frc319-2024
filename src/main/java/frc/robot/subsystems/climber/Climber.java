// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP", 0.0);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Climber/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Climber/kD", 0.0);
  private static final LoggedTunableNumber kFF = new LoggedTunableNumber("Climber/kFF", 0.0);

  private static LoggedTunableNumber climberSetpoint = new LoggedTunableNumber("Climber/setpoint", 0.0);
  private static double climberPosition = 0.0;

  public Climber(ClimberIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
      case SIM:
      default:

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber",inputs);
  }

  public void stop() {
    io.stop();
  }

  public void configurePID(double kP, double kI, double kD, double kFF) {
    io.configurePID(kP, kI, kD, kFF);
  }

  public void setPosition(double targetPosition) {
    io.setPosition(targetPosition);
  }

  public double getVelocity() {
    return io.getVelocity();
  }

  public double getPosition() {
    return io.getPosition();
  }  

  public double getCurrent() {
    return io.getCurrent();
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void setLeftPO(double leftPO) {
    io.setLeftPO(leftPO);
  }

    public void setRightPO(double rightPO) {
    io.setRightPO(rightPO);
  }
}
