// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.shooter.Shooter;

public class Collect extends Command {

  Shooter m_shooter;
  Collector m_collector;

  /** Creates a new Collect. */
  public Collect(Shooter shooter , Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_collector = collector;
    addRequirements(shooter, collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // 1. Shooter Flywheels should stop
    m_shooter.runShooterVelocity(0);

    // 2. Shooter should go to a Preload position
    m_shooter.setWristPosition(WristConstants.Setpoints.home);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 1. If the note is detected by the first beam break
    if(m_collector.isBeamBreakTripped() == false) {

      // NO - We need to collect the note. 'Collector' and 'Tunnel' Rollers can now Intake
      m_collector.setRollersPO(1.0);
    } else {
      // YES - We have the note at the end of the tunnel. 'Collector' and 'Tunnel' rollers should stop 
      m_collector.setRollersPO(0.0);
      Robot.leds.colorTest(255, 30, 0); //orange or blue 
     // Robot.leds.colorTest(0, 0, 255);

      
    }
    // until ...
    // If the note is detected by the first beam break, and the Shooter is in the right position  
    if(m_collector.isBeamBreakTripped() ) {
      // True : The passoff can be completed. 'Tunnel' and 'Feed' Rollers 
      m_collector.setTunnelRollersPO(1.0);
      m_shooter.setFeedPO(1.0);

    }else {
      // False : We need to do nothing while we wait for the shooter wrist to move
      m_collector.setRollersPO(0.0);
      m_shooter.setFeedPO(0.0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Collector should stop
    m_collector.setRollersPO(0.0);
    // Feed should stop
    m_shooter.setFeedPO(0.0);

    // Shooter should go to the default / furthest shot position
    m_shooter.setWristPosition(WristConstants.Setpoints.shoot);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // This command is finished when the note is detected by the second beam break 
    return m_shooter.isBeamBreakTripped();
  }
}
