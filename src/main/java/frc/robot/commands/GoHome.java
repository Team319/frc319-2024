// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.shooter.Shooter;

public class GoHome extends Command {
    Shooter m_shooter;
    Elevator m_elevator;
    double wristThreshold;
    double elevatorThreshold;
    /** Creates a new GoHome. */
  public GoHome(Shooter shooter, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_shooter = shooter;
  m_elevator = elevator;
  wristThreshold = 0.015;
  addRequirements(shooter, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setWristPosition(WristConstants.Setpoints.home);
    m_elevator.setPosition(ElevatorIOReal.ElevatorSetpoint.BOTTOM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_shooter.setWristPosition(WristConstants.Setpoints.home);
    m_elevator.setPosition(ElevatorIOReal.ElevatorSetpoint.BOTTOM);
 }
   


  // Called once the command ends or is interrupted.
  @Override

  public void end(boolean interrupted) {
    m_shooter.setWristPosition(WristConstants.Setpoints.home);
    m_elevator.setPosition(ElevatorIOReal.ElevatorSetpoint.BOTTOM);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
