// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class Elevator extends SubsystemBase {
  
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.0);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0);
  private static final LoggedTunableNumber kFF = new LoggedTunableNumber("Elevator/kFF", 0.0);

  private static LoggedTunableNumber elevatorSetpoint = new LoggedTunableNumber("Elevator/setpoint", 0.0);
  private static double elevatorPosition = 0.0;

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
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
    Logger.processInputs("Elevator",inputs);

    // since we update inputs, we can read these values from AdvantageScope

    //Logger.recordOutput("Elevator/Position",getPosition());

    LoggedTunableNumber.ifChanged(hashCode(), setpoint-> setPosition(setpoint[0]), elevatorSetpoint);

    LoggedTunableNumber.ifChanged(hashCode(), pid -> configurePID(pid[0], pid[1], pid[2], pid[3]) ,kP, kI, kD, kFF);


   //System.out.println(elevatorPosition + getPosition());

  }

  public void stop() {
    io.stop();
  }

  public void setPO(double PO) {
    io.setPO(PO);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void configurePID(double kP, double kI, double kD, double kFF) {
    io.configurePID(kP, kI, kD, kFF);
  }

  public double getPosition() {
    return io.getPosition();
  }

  public double getVelocity() {
    return io.getVelocity();
  }
}
