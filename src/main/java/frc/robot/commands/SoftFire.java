// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.shooter.Shooter;

public class SoftFire extends Command {
  Shooter m_shooter;
  Collector m_collector;
  Elevator m_elevator;
  int passedCycles;
  double setpoint;
  double threshold;
  double wristThreshold;
  double elevatorThreshold;
  /** Creates a new Fire. */
  public SoftFire(Shooter shooter, Collector collector, Elevator elevator, double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_collector = collector;
    m_elevator = elevator;
    setpoint = RPM;
    threshold = 75;
    wristThreshold = 0.015;
    elevatorThreshold = 0.5;

    addRequirements(shooter, collector);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShooterVelocity(setpoint);
    m_shooter.setFeedPO(0.0);
   // System.out.println("init");
    passedCycles = 0;
    m_shooter.setWristPosition(WristConstants.Setpoints.bloop);
    m_collector.setTunnelRollersPO(0.35); //0.2
    m_elevator.setPosition(ElevatorIOReal.ElevatorSetpoint.AMP);


  }
 
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (m_shooter.getWristPosition() > WristConstants.Setpoints.bloop-wristThreshold && m_shooter.getWristPosition() < WristConstants.Setpoints.bloop+wristThreshold){
      System.out.println("Ding");
      System.out.println("RPM"+m_shooter.getVelocityRPM());
    if (m_elevator.getPosition() > ElevatorIOReal.ElevatorSetpoint.AMP-elevatorThreshold && m_elevator.getPosition() < ElevatorIOReal.ElevatorSetpoint.AMP+elevatorThreshold){
        System.out.println("Dong");
    if (m_shooter.getVelocityRPM() > setpoint-threshold && m_shooter.getVelocityRPM() < setpoint+threshold) {
        System.out.println("Dong");
        m_collector.setTunnelRollersPO(0.4);
        m_shooter.setFeedPO (1.0);
        passedCycles++;
        System.out.println("passedCycles"+passedCycles);

    }
   }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_shooter.setFeedPO(0.0);
    m_elevator.setPosition(ElevatorIOReal.ElevatorSetpoint.BOTTOM);
    //System.out.println("end");
    m_collector.setTunnelRollersPO(0.0);
    m_shooter.setWristPosition(WristConstants.Setpoints.home);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return passedCycles >= 10; 
}
}
