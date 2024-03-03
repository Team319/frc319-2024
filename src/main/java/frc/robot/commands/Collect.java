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
  int passedCycles = 0;
  boolean firstDetectionOccured = false;

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

    passedCycles = 0;
    System.out.println("init");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 1. If the note is detected by the first beam break
    if(m_collector.isBeamBreakTripped() == false && firstDetectionOccured == false) {
      System.out.println("1. Not tripped");

      // NO - We need to collect the note. 'Lower' and 'Tunnel' Rollers can now Intake
      m_collector.setRollersPO(0.3);
    } else {
      System.out.println(" 1. tripped!");
      firstDetectionOccured = true;
      // YES - We have the note at the end of the tunnel. 'Lower' rollers should stop 
      m_collector.setLowerRollersPO(0.0);
      
    }
    // until ...

    // If the note is detected by the first beam break, and the Shooter is in the right position  

    if(firstDetectionOccured){
      if(m_collector.isBeamBreakTripped() ) {
      // True : The passoff can be completed. 'Tunnel' and 'Feed' Rollers 

      System.out.println("2. still tripped... trying to pass off");
      m_collector.setTunnelRollersPO(0.3);
      m_shooter.setFeedPO(0.5);

    }else {
      // False : We need to do nothing while we wait for the shooter wrist to move
      System.out.println("2. beam break not tripped ... incrementing cycles" + passedCycles );

      passedCycles++;
    }
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(" end!");
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
    return passedCycles > 10;
  }
}
