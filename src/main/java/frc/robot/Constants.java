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

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>9It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;
  public static final boolean tuningMode = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /* 319's swerve chassis */
    BUSTER,

    /** Running a real robot with a tank drive. */
    TANK,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum HeadingTargets{
    NO_TARGET,
    SPEAKER,
    SOURCE
  }

  public static class TargetLocations{
    public static Translation2d ORIGIN = new Translation2d();
    public static Translation2d RED_SPEAKER = new Translation2d(16.45,5.3);
    public static Translation2d BLUE_SPEAKER = new Translation2d(0.0,5.3);
    public static Translation2d RED_SOURCE = new Translation2d(0.0,-0.5);
    public static Translation2d BLUE_SOURCE = new Translation2d(16.15,-0.5);
  }

  public static class WristConstants {
        public static class PID {
          public static final double kP = 3.75; //BSU 1.75 3.75
          public static final double kI = 0; //0.0005
          public static final double kD = 0;
  
          public static final int iZone = 0;
          public static final double kFF = 0.0;
        }
  
        public static class Setpoints {
          public static final double top = 0.485; //.4
          public static final double shoot = 0.3;
          public static final double home = 0.0083; //0.025
          public static final double sub =0.083; //.08 worked
          public static final double podium = 0.18; //.185
          public static final double amp =0.475;
          public static final double bottom = 0.0083; 
        }
  
        public static class SoftLimits {
          public static final float forwardSoftLimit = (float)Setpoints.top;
          public static final float reverseSoftLimit = (float)Setpoints.bottom;
        }
  
      }

    public static class ShooterConstants {
      public static class PID {
        public static final double kP = 1.0;
        public static final double kI = 0.7; // 0.5 also worked
        public static final double kD = 0;

        public static final int iZone = 0;
        public static final double kFF = 0.0;
      }

      public static class Setpoints {
        public static final double top = 0.625; //.4
        public static final double shoot = 0.3;
        public static final double home = 0.232; //0.025
        public static final double podium = 0.375;
        public static final double amp = 0.671;
        public static final double bottom = 0.025; 
      }

      public static class SoftLimits {
        public static final float forwardSoftLimit = (float)Setpoints.top;
        public static final float reverseSoftLimit = (float)Setpoints.bottom;
      }

    }

  public static class ElevatorConstants{
    public static class PID {
      public static final double kPUp = 0.2;
      public static final double kIUp = 0;
      public static final double kDUp = 0;
      public static final double kFFUp = 0.0;
      public static final int iZoneUp = 0;

      public static final double kPDown = kPUp;
      public static final double kIDown = kIUp;
      public static final double kDDown = kDUp;
      public static final double kFFDown = kFFUp;
      public static final int iZoneDown = iZoneUp;
      
    }

    public static class Setpoints {
      public static final float top = (float)99.0;// FYI: Real top is closer to 130-135 
      public static final float trap = top;
      public static final float climb = top;
      public static final float amp = (float)47.785;
      public static final float bottom = (float)2.0; // leaves some space for the elevator to settle without tapping bottom of travel
      public static final float shoot = bottom;

    }

    public static class SoftLimits {
      public static final float forwardSoftLimit = Setpoints.top;
      public static final float reverseSoftLimit = Setpoints.bottom;
    }

  }

  public static class ClimberConstants {
    public static class PID {
      public static final double kPUp = 0.0;  // Power applied to motor
      public static final double kIUp = 0.0;  // margin of error in motor
      public static final double kDUp = 0.0;  // Makes the graph line smooth from point A to point B
      public static final double kFFUp = 0.0; // Feedforward value
      public static final int iZoneUp = 0;

      public static final double kPDown = 0.0;  // Power applied to motor
      public static final double kIDown = 0.0;  // margin of error in motor
      public static final double kDDown = 0.0;  // Makes the graph line smooth from point A to point B
      public static final double kFFDown = 0.0; // Feedforward value
      public static final int iZoneDown = 0;
    }

    public static class SoftLimits {
      public static final float forwardSoftLimit = Setpoints.top;
      public static final float reverseSoftLimit = Setpoints.bottom;
    }

    public static class Setpoints {
      public static final float top = (float)85.0;
      
      public static final float bottom = (float)-45.0;
    }

  }

  public static class LimelightConstants{
    public static enum Device{
      SHOOTER,
      COLLECTOR
    }
  }

      
}
