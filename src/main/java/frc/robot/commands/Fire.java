// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.HelperFunctions;

public class Fire extends Command {
  Drive m_drive;
  Shooter m_shooter;
  Collector m_collector;

  int passedCycles;
  double setpoint;
  double threshold;
  double wristThreshold;
  boolean firstDetectionOccured = false;
  /** Creates a new Fire. */
  public Fire(Drive drive, Shooter shooter, Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive; 
    m_shooter = shooter;
    m_collector = collector;
    passedCycles = 0;
    firstDetectionOccured = false;
    setpoint = 4750;
    threshold = 75; 
    wristThreshold = 0.015;
    
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShooterVelocity(setpoint);
    m_shooter.setFeedPO(0.0);
    //System.out.println("Fire init");
    passedCycles = 0;
    firstDetectionOccured = false;
    m_drive.setUpdatePoseWithVision(true);
    m_collector.setTunnelRollersPO(0.5);
  }
 
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(m_shooter.isBeamBreakTripped() == false && firstDetectionOccured == false) {

    } else {
      firstDetectionOccured = true;
    } 

    System.out.println(" is wrist happy? " +  HelperFunctions.isWithin(m_shooter.getWristPosition(), m_shooter.getWristSetpointForDistance(m_drive.getDistanceToAllianceSpeaker()), wristThreshold));
    if (m_shooter.getWristPosition() > WristConstants.Setpoints.sub1-wristThreshold && m_shooter.getWristPosition() < WristConstants.Setpoints.sub1+wristThreshold){
      { 
    /*if (firstDetectionOccured)
    { 
    if (m_shooter.isBeamBreakTripped()){
*/
      System.out.println(" is rpm happy? " + HelperFunctions.isWithin(m_shooter.getVelocityRPM(), setpoint, threshold));
      if ( HelperFunctions.isWithin(m_shooter.getVelocityRPM(), setpoint, threshold) ) //( m_shooter.getVelocityRPM() > setpoint-threshold && m_shooter.getVelocityRPM() < setpoint+threshold) 
      {
        m_shooter.setFeedPO (1.0);
        
       // System.out.println(" Is Shooter Empty?: " + !m_shooter.isBeamBreakTripped());
       // if( !m_shooter.isBeamBreakTripped() ) {
        passedCycles++;
      //  System.out.println("passedCycles"+passedCycles);
       // }
      }
    }
   }
  }
  //}
//}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_shooter.stop();
    m_shooter.setWristPosition(WristConstants.Setpoints.home);
    m_shooter.setFeedPO(0.0);
    m_drive.setUpdatePoseWithVision(false);
    m_collector.setTunnelRollersPO(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return passedCycles >= 5;
  }
}
