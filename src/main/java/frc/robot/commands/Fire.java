// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.HelperFunctions;

public class Fire extends Command {
  Drive m_drive;
  Shooter m_shooter;

  int passedCycles;
  double setpoint;
  double threshold;
  double wristThreshold;
  boolean firstDetectionOccured = false;
  /** Creates a new Fire. */
  public Fire(Drive drive, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive; 
    m_shooter = shooter;
    passedCycles = 0;
    firstDetectionOccured = false;
    setpoint = 5000;
    threshold = 550;
    wristThreshold = 0.015;
    addRequirements(shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShooterVelocity(setpoint);
    m_shooter.setFeedPO(0.0);
    System.out.println("init");
    passedCycles = 0;
    firstDetectionOccured = false;
m_drive.setUpdatePoseWithVision(true);

    

  }
 
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //Potentially redundant, but we want to make sure the wrist is at the right position
    if(m_shooter.isBeamBreakTripped() == false && firstDetectionOccured == false) {

    m_shooter.setWristPosition(m_shooter.getWristSetpointForDistance(m_drive.getDistanceToAllianceSpeaker()));
    } else {
      firstDetectionOccured = true;
    } 

    //System.out.println("RPM" + m_shooter.getVelocityRPM());
    
    if ( HelperFunctions.isWithin(m_shooter.getWristPosition(), m_shooter.getWristSetpointForDistance(m_drive.getDistanceToAllianceSpeaker()), wristThreshold) )
    { 
    /*if (firstDetectionOccured)
    { 
    if (m_shooter.isBeamBreakTripped()){
*/
      if ( HelperFunctions.isWithin(m_shooter.getVelocityRPM(), setpoint, threshold) ) //( m_shooter.getVelocityRPM() > setpoint-threshold && m_shooter.getVelocityRPM() < setpoint+threshold) 
      {
        m_shooter.setFeedPO (1.0);
        passedCycles++;
        System.out.println("passedCycles"+passedCycles);

      }
    }
  }
  //}
//}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_shooter.setWristPosition(WristConstants.Setpoints.home);
    m_shooter.setFeedPO(0.0);
    m_drive.setUpdatePoseWithVision(false);
   

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return passedCycles >= 10;
  }
}
