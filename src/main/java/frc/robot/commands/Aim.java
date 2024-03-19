// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HeadingTargets;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.HelperFunctions;

public class Aim extends Command {
  Drive m_drive;
  Shooter m_shooter;
  double distanceToSpeaker = 0.0;
  boolean isWristAtDesiredSetpoint = false;
  double wristThreshold = 0.015;
  /** Creates a new Aim. */
  public Aim(Drive drive , Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_shooter = shooter;
    addRequirements(shooter);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Drivetrain : Begin Tracking a Target

    // Pre-spin the Shooter Flywheels to some speed
    //m_shooter.setShooterVelocity(3000); //TODO Find optimal speed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 1. If I'm within some Distance to the Target
    distanceToSpeaker = m_drive.getDistanceToAllianceSpeaker();
    if(distanceToSpeaker <= 5.0){ // TODO : Tune this, what's our furthest shot. ie - Podium shot is 2.286 meters 
      
      // Update the Shooter Flywheel Velocity 
      //m_shooter.setShooterVelocity(5000); //TODO Find optimal shot speed

      // Update the Shooter Wrist Position
      m_shooter.setWristPosition(m_shooter.getWristSetpointForDistance(m_drive.getDistanceToAllianceSpeaker()));
    }
    
    // Command the drivetrain to rotate to face the speaker
    m_drive.setHeadingTarget(HeadingTargets.SPEAKER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    isWristAtDesiredSetpoint =  HelperFunctions.isWithin(m_shooter.getWristPosition(), m_shooter.getWristSetpointForDistance(m_drive.getDistanceToAllianceSpeaker()), wristThreshold);
    return isWristAtDesiredSetpoint;
  }
}
