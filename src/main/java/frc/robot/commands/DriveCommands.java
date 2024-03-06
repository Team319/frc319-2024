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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PolarCoordinate;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.2;
  private static final double HEADING_DEADBAND = DEADBAND;
  private static final double JOYSTICK_GOVERNOR = 0.5; // this value must not exceed 1.0
  private static final double THROTTLE_GOVERNOR = 1.0 - JOYSTICK_GOVERNOR;

  private DriveCommands() {

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
      DoubleSupplier throttleSupplier) {

    switch(Constants.currentMode){
      case TANK:
        return Commands.run(
        () -> {  drive.tankDrive( ySupplier.getAsDouble(), headingXSupplier.getAsDouble()); },
         drive);

      case REAL:
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

            double omega = MathUtil.applyDeadband(headingXSupplier.getAsDouble(), HEADING_DEADBAND); 

            double throttle = MathUtil.applyDeadband(throttleSupplier.getAsDouble(), DEADBAND);


            // Square values and apply relative throttle
            // Note : we don't care about the magnitude of the throttle, as we have the
            // linearDirection to apply later
            // Note : only apply throttle if we have provided a linear magnitude

            if (throttle > 0.25) {
              linearMagnitude *= THROTTLE_GOVERNOR;
            }
            
            linearMagnitude = (linearMagnitude * linearMagnitude);

            /* 
            // Old way. Trigger implemented as throttle, instead of a slow mode 
            // We swapped because it hurts the driver's wrist to hold the trigger the majority of the time
            if (linearMagnitude > 0.0 && throttle > 0.0) {
              linearMagnitude +=
                  Math.copySign(throttleSupplier.getAsDouble() * THROTTLE_GOVERNOR, linearMagnitude);
            }*/

            
            omega = drive.snapToHeading(headingXSupplier, headingYSupplier);
            
            if (drive.getHeadingTarget() != Drive.HeadingTargets.NO_TARGET){
              omega = drive.snapToTarget();
            }

            // Calcaulate new linear velocity
            Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                    .getTranslation();

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
