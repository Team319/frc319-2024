// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HeadingTargets;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.HelperFunctions;

public class AimInAuto extends Command {
  Drive m_drive;
  Shooter m_shooter;
  Collector m_collector;
  double distanceToSpeaker = 0.0;
  boolean isWristAtDesiredSetpoint = false;
  double wristThreshold = 0.015;
  /** Creates a new Aim. */
  public AimInAuto(Drive drive , Shooter shooter, Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_shooter = shooter;
    m_collector = collector;

    addRequirements(shooter);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      System.out.println("Aim In Auto: Init");
      m_drive.setUpdatePoseWithVision(true);
      m_collector.setTunnelRollersPO(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 1. If I'm within some Distance to the Target
    distanceToSpeaker = m_drive.getDistanceToAllianceSpeaker();
   // if(distanceToSpeaker <= 5.0){ // TODO : Tune this, what's our furthest shot. ie - Podium shot is 2.286 meters 
      
      // Update the Shooter Flywheel Velocity 
      //m_shooter.setShooterVelocity(5000); //TODO Find optimal shot speed

      // Update the Shooter Wrist Position
     m_shooter.setWristPosition(m_shooter.getWristSetpointForDistance(m_drive.getDistanceToAllianceSpeaker()));
    //}
    
    // Command the drivetrain to rotate to face the speaker
    m_drive.setHeadingTarget(HeadingTargets.SPEAKER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   // m_drive.setUpdatePoseWithVision(false);
    m_collector.setTunnelRollersPO(0.0);
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    isWristAtDesiredSetpoint =  HelperFunctions.isWithin(m_shooter.getWristPosition(), m_shooter.getWristSetpointForDistance(m_drive.getDistanceToAllianceSpeaker()), wristThreshold);
    return isWristAtDesiredSetpoint;
  }
}
