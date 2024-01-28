// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.collector;

/** Add your docs here. */
public interface CollectorIO {

    public default void stop() {}

    public default void setCollectorPO(double PO) {}

    public default void detectGamePiece() {}

    public default void setCollectorPosition(double position) {}

    public default void setTunnelPO(double PO) {}

}
