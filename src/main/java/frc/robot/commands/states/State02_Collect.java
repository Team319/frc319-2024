// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.HelperFunctions;

public class State02_Collect extends Command {
  Shooter m_shooter;
  Collector m_collector;
  Elevator m_elevator;
  
  int passedCycles = 0;
  boolean firstDetectionOccured = false;
  double wristThreshold;
  
  /** Creates a new Collect. */
  public State02_Collect(Drive drive, Shooter shooter , Collector collector, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_collector = collector;
    m_elevator = elevator;
    passedCycles = 0;
    firstDetectionOccured = false;
    wristThreshold = 0.1;
    addRequirements(shooter, collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

    // TODO: Update Robot state variable so outside methods can see state


    // 1. Stop Shooter Flywheels 
    //m_shooter.setShooterVelocity(0);

    // 2. Shooter Wrist should go to a Preload position
    m_shooter.setWristPosition(WristConstants.Setpoints.home);
    m_elevator.setPosition(ElevatorConstants.Setpoints.bottom);

    passedCycles = 0;
    firstDetectionOccured = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    // If the wrist is within the threshold of collecting setpoint, we can start collecting
    if ( HelperFunctions.isWithin(m_shooter.getWristPosition() , WristConstants.Setpoints.home , wristThreshold) )
    {

      if( m_collector.isBeamBreakTripped() == false && firstDetectionOccured == false) {
        // NO - We need to collect the note. 'Lower' and 'Tunnel' Rollers can now Intake
        m_collector.setRollersPO(1.0);
      
      }
      else
      {
        //System.out.println(" 1. tripped!");
        firstDetectionOccured = true;
        // YES - We have the note at the end of the tunnel. 'Lower' rollers should stop 
        m_collector.setLowerRollersPO(0.0);

      }
    } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
