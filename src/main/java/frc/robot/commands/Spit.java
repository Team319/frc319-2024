// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.shooter.Shooter;

public class Spit extends Command {
  Shooter m_shooter;
  Collector m_collector;
  Elevator m_elevator;
  double setpoint;
  double wristThreshold;
  double elevatorThreshold;

  /** Creates a new Spit. */
  public Spit(Shooter shooter, Collector collector, Elevator elevator, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_collector = collector;
    m_elevator = elevator;
    setpoint = RPM;
    wristThreshold = 0.015;
    elevatorThreshold = 0.015;

    addRequirements(shooter, collector, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_collector.setRollersPO(-1.0);
    m_shooter.setWristPosition(WristConstants.Setpoints.home);
    m_elevator.setPosition(ElevatorIOReal.ElevatorSetpoint.BOTTOM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setWristPosition(WristConstants.Setpoints.home);
    m_elevator.setPosition(ElevatorIOReal.ElevatorSetpoint.BOTTOM);
    m_shooter.setFeedPO(-1.0);
  }
   
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_collector.setRollersPO(0.0);
    m_shooter.stop();
    m_shooter.setFeedPO(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
