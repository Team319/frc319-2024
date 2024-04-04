// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.shooter.Shooter;

public class FireTrap extends Command {
  Shooter m_shooter;
  Collector m_collector;
  int passedCycles;
  double setpoint;
  double threshold;
  double wristThreshold;
  /** Creates a new Fire. */
  public FireTrap(Shooter shooter, Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_collector = collector;
    wristThreshold = 0.015; //15
    addRequirements(shooter, collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShooterVelocity(setpoint);
    m_shooter.setFeedPO(0.0);
    System.out.println("init");
    passedCycles = 0;
    m_shooter.setWristPosition(WristConstants.Setpoints.top);

  }
 
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    if (m_shooter.getWristPosition() > WristConstants.Setpoints.top-wristThreshold && m_shooter.getWristPosition() < WristConstants.Setpoints.top+wristThreshold){
        m_shooter.setFeedPO (-1.0);
        m_shooter.getVelocityRPM();

        passedCycles++;
        System.out.println("passedCycles"+passedCycles);

      }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_shooter.setFeedPO(0.0);
    System.out.println("end");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return passedCycles >= 10;
  }
}
