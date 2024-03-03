// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.shooter.Shooter;

public class Spit extends Command {
  Shooter m_shooter;
  Collector m_collector;

  /** Creates a new Spit. */
  public Spit(Shooter shooter, Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_collector = collector;
    addRequirements(shooter, collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_collector.setRollersPO(-1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_collector.setRollersPO(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
