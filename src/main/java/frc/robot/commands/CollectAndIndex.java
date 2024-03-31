// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.HelperFunctions;

public class CollectAndIndex extends Command {

  private static enum CollectorStates {
    NO_NOTE,
    FIRST_DETECTION,
    GOING_UP,
    GOING_DOWN,
    GOING_FINAL,
    AT_FINAL
  } 

  Shooter m_shooter;
  Collector m_collector;
  int passedCycles = 0;
  boolean initialDetectionOccured = false;
  boolean atFinalCompleted = false;
  double wristThreshold;

  CollectorStates collectState = CollectorStates.NO_NOTE;

  /** Creates a new Collect. */
  public CollectAndIndex(Shooter shooter , Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_collector = collector;
    passedCycles = 0;
    initialDetectionOccured = false;
    atFinalCompleted = false;
    wristThreshold = 0.1;
    collectState = CollectorStates.NO_NOTE;
    addRequirements(shooter, collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // 1. Shooter Flywheels should stop
    //m_shooter.setShooterVelocity(0);

    // 2. Shooter should go to a Preload position
    m_shooter.setWristPosition(WristConstants.Setpoints.home);

    passedCycles = 0;
    initialDetectionOccured = false;

    collectState = CollectorStates.NO_NOTE;
    atFinalCompleted = false;
    //System.out.println("init");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 1. If the note is detected by the first beam break
   //System.out.println("Wrist "+m_shooter.getWristPosition());
    if (HelperFunctions.isWithin(m_shooter.getWristPosition(), WristConstants.Setpoints.home, wristThreshold)){ 

      if(m_collector.isBeamBreakTripped() == false && initialDetectionOccured == false) {
      //System.out.println("1. Not tripped");

      // NO - We need to collect the note. 'Lower' and 'Tunnel' Rollers can now Intake
      m_collector.setRollersPO(1.0);
    } else {
      //System.out.println(" 1. tripped!");
      initialDetectionOccured = true;
      // YES - We have the note at the end of the tunnel. 'Lower' rollers should stop 
      m_collector.setLowerRollersPO(0.0);
      
    }
    
    switch (collectState) {

      case NO_NOTE:
      default:
        collectState = CollectorStates.NO_NOTE;
        m_collector.setRollersPO(1.0);

        if(m_collector.isBeamBreakTripped()){
          collectState = CollectorStates.FIRST_DETECTION;
        }
        break;

      case FIRST_DETECTION:
        m_collector.setLowerRollersPO(0.0);
        m_collector.setTunnelRollersPO(0.4);
        m_shooter.setFeedPO(0.25);

        if (m_shooter.isBeamBreakTripped()){
          collectState = CollectorStates.GOING_UP;
        }
        break;

      case GOING_UP:

        m_collector.setLowerRollersPO(0.0);
        m_collector.setTunnelRollersPO(0.4);
        m_shooter.setFeedPO(0.25);

        if ( !m_collector.isBeamBreakTripped() ){
          collectState = CollectorStates.GOING_DOWN;
        }
        break;

      case GOING_DOWN:

        m_collector.setLowerRollersPO(0.0);
        m_collector.setTunnelRollersPO(-0.4);
        m_shooter.setFeedPO(-0.25);

        if ( m_collector.isBeamBreakTripped() && !m_shooter.isBeamBreakTripped() ){
          collectState = CollectorStates.GOING_FINAL;
        }
        break;

      case GOING_FINAL:
        m_collector.setLowerRollersPO(0.0);
        m_collector.setTunnelRollersPO(0.3);
        m_shooter.setFeedPO(0.20);

        if ( m_shooter.isBeamBreakTripped() ){
          collectState = CollectorStates.AT_FINAL;
        }
        
        break;
      case AT_FINAL:
        atFinalCompleted = true;
        
        break;
    
      
    }

  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println(" end!");
    // Collector should stop
    m_collector.setRollersPO(0.0);
    // Feed should stop
    m_shooter.setFeedPO(0.0);
    // Pre-spin shooter wheels
    //m_shooter.setShooterVelocity(3000);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // This command is finished when the note is detected by the second beam break 
    return collectState == CollectorStates.AT_FINAL;
  }
}
