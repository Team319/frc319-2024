// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.limelight;

/* import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.GenericEntry;*/
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
/*import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;*/
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;

public class Limelight extends SubsystemBase {

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight"); // Makes the Limelight data table.
  private NetworkTableEntry m_botPose = m_table.getEntry("botpose"); // Makes a double array to hold the robot pose.
  private NetworkTableEntry m_tv = m_table.getEntry("tv");
  private NetworkTableEntry m_tx = m_table.getEntry("tx");
  private NetworkTableEntry m_ty = m_table.getEntry("ty");
  private NetworkTableEntry m_ta = m_table.getEntry("ta");
  private NetworkTableEntry m_tl = m_table.getEntry("tl");
  private NetworkTableEntry m_cl = m_table.getEntry("cl");

 // private NetworkTableEntry m_

  /** Creates a new Limelight. */
  public Limelight() {

  }

  public double getLatency() {
    return m_tl.getDouble(0.0); //Returns latency

  }
  public double getTotalLatency() {
    return m_cl.getDouble(0.0); //Returns total latency

   }
   public double getTargetArea(){
    return m_ta.getDouble(0.0); // Returns the size of a valid target (how close you are to the target)
  }

  public double getHorizontalOffset() {
    return m_tx.getDouble(0.0); // Returns the horizontal offset from valid target
  }

  public double getVerticalOffset() {
    return m_ty.getDouble(0.0); // Returns the vertical offset from valid target
  }

  public double getDistance() {
    return m_botPose.getDouble(0.0); // TODO: Returns the current distance.
  }

  public boolean isValidTargetSeen () {
    return m_tv.getBoolean(false); //Returns if a valid target is seen or not
  }

  public double[] getBotPose() {
    return  m_botPose.getDoubleArray(new double[6]); //Returns field space robot pose
  }

  @Override
  public void periodic() {
    // Do nothing at this time  
  }
}






