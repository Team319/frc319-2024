// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.shooter.Shooter;

public class Aim extends Command {
  Shooter m_shooter;
  Collector m_collector;
  /** Creates a new Aim. */
  public Aim(Shooter shooter, Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_collector = collector;
    addRequirements(shooter, collector);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Drivetrain : Begin Tracking a Target

    // Pre-spin the Shooter Flywheels to some speed
    m_shooter.setShooterVelocity(0); //TODO Find optimal speed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 1. If I'm within some Distance to the Target

      // Update the Shooter Flywheel Velocity 
      m_shooter.setShooterVelocity(0); //TODO Find optimal speed

      // Update the Shooter Wrist Position
      m_shooter.setWristPosition(WristConstants.Setpoints.shoot);
      
    // ??? - Update the Drivetrain to point to the Target?

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
