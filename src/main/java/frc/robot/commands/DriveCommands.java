// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.HeadingTargets;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.2;
  private static final double HEADING_DEADBAND = DEADBAND;
  private static final double JOYSTICK_GOVERNOR = 0.5; // this value must not exceed 1.0
  //private static final double THROTTLE_GOVERNOR = 1.0 - JOYSTICK_GOVERNOR;

  private static final boolean isSnapHeadingWithJoystickEnabled = false;

  private DriveCommands() {

  }

  public static Command lockHeadingToSpeaker(Drive drive){
    return Commands.run(
      ()-> { drive.setHeadingTarget(HeadingTargets.SPEAKER);
        drive.setUpdatePoseWithVision(true); }
    ) ;
  }

  public static Command unlockHeading(Drive drive){
    return Commands.run(
      ()-> { drive.setUpdatePoseWithVision(false); }
    ) ;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier headingXSupplier,
      DoubleSupplier headingYSupplier,
      DoubleSupplier throttleSupplier) 
      {

        switch(Constants.currentMode)
        {
          case TANK:
            return Commands.run(
            () -> {  drive.tankDrive( ySupplier.getAsDouble(), headingXSupplier.getAsDouble()); },
            drive);

          case REAL:
          case BUSTER:
          case SIM:
          case REPLAY:
          return Commands.run(
              () -> {
                // Apply deadband
                double linearMagnitude =
                    MathUtil.applyDeadband(
                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                        DEADBAND); // get the magnitude of the joystick

                Rotation2d linearDirection =
                    new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                double omega = MathUtil.applyDeadband(headingYSupplier.getAsDouble(), HEADING_DEADBAND); 

                double throttle = MathUtil.applyDeadband(throttleSupplier.getAsDouble(), DEADBAND);


                // Square values and apply relative throttle
                // Note : we don't care about the magnitude of the throttle, as we have the
                // linearDirection to apply later
                // Note : only apply throttle if we have provided a linear magnitude


                // Slow mode if Trigger is pulled
                if(throttle > 0.25){
                  linearMagnitude = linearMagnitude * linearMagnitude * JOYSTICK_GOVERNOR;
                }
                else{
                  linearMagnitude = linearMagnitude * linearMagnitude;
                }

                

                if (isSnapHeadingWithJoystickEnabled) // Maintain target heading from joystick
                {
                  omega = drive.snapToHeading(headingXSupplier, headingYSupplier);
                
                  if (drive.getHeadingTarget() != HeadingTargets.NO_TARGET)
                  {
                    omega = drive.snapToTarget();
                  }
                
                  if(drive.getHeadingTarget() != HeadingTargets.NO_TARGET) // Maintain target heading
                  { // Heading locked to a target
                    omega = drive.snapToTarget(); 
                  }
                }
                else // Perform manual rotation with joystick input
                { 
                  
                  if( omega != 0 || drive.isHeadingLocked() == false ) // Right joystick input greater than the deadband was provided, or heading isn't locked
                  {
                    drive.unlockHeading();
                    
                    if ( throttle > 0.25) 
                    {
                      omega = Math.copySign((omega * omega * JOYSTICK_GOVERNOR), omega);
                    }
                    else
                    {
                      omega = Math.copySign((omega * omega ), omega);
                    }
                  }
                  else   // No joystick input, but heading is locked
                  {
                    // Maintain some heading
                    if(drive.getHeadingTarget() != HeadingTargets.NO_TARGET) 
                    { // Heading is locked to a target
                      omega = drive.snapToTarget(); 
                    }
                    else
                    { // Heading is locked to a setpoint
                      omega = drive.snapToHeading();
                    }
                    
                  }

                }
                
                // Calcaulate new linear velocity
                Translation2d linearVelocity =
                    new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

                if ( DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
                    linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
                }

                // Convert to field relative speeds
                ChassisSpeeds fieldRelativeVelocities = ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega * drive.getMaxAngularSpeedRadPerSec(),
                        drive.getRotation());

                
                // Discretize & send command
                drive.runVelocity( ChassisSpeeds.discretize(fieldRelativeVelocities, 0.02));
              },
              drive);

          default:
            return Commands.run(
            () -> {} /* do nothing */,
            drive);
        }
      }
}
