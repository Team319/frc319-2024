// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;

public class State04_Aim extends Command {
  /** Creates a new Aim. */
  public State04_Aim() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: Update Robot state variable so outside methods can see state

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Calculate/Get current Distance to target

    // Update Shooter Pivot Position Setpoint for appropriate distance

    // Update Shooter Flywheel Velocity Setpoint for appropriate distance

    // if shooter Pivot and Flywheel position/velocity are within setpoint threshold

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
