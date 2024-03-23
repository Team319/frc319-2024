// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class JoystickClimb extends Command {
  private static final double DEADBAND = 0.5;
  /** Creates a new JoystickClimb. */

  Climber m_climber;
  double rightPO = 0.0;
  double leftPO = 0.0;

  public JoystickClimb(Climber climber,DoubleSupplier leftSupplier,DoubleSupplier rightSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    addRequirements(climber);
    leftPO = MathUtil.applyDeadband(leftSupplier.getAsDouble(), DEADBAND);
    rightPO = MathUtil.applyDeadband(rightSupplier.getAsDouble(), DEADBAND);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setLeftPO(leftPO);
    m_climber.setRightPO(rightPO);

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
