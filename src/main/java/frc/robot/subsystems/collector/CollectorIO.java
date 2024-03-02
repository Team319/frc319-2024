// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface CollectorIO {

    @AutoLog
    public static class CollectorIOInputs {
        public int maxCurrentAmps = 0;
        public double appliedPercentOutput = 0.0;
        public boolean isLeadMotorInverted = false;
        public double outputCurrentAmps = 0.0;

        public double feedRollerAmps = 0.0;
    }

    public default void updateInputs(CollectorIOInputs inputs) {}

    public default void stop() {}

    public default void setRollersPO(double collectorPO) {}

    public default void setTunnelPO(double collectorPO) {}

    public default void setLowerPO(double collectorPO) {}

    public default void detectGamePiece() {}

    public default void setRollersPOsition(double position) {}

    public default void setFeedVoltage(double ffVolts) {}

    public default boolean isBeamBreakTripped() { return false; }

}
