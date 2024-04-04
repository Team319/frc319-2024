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
//import frc.robot.Constants;
import frc.robot.Constants.LimelightConstants;

public class Limelight {

  private static final NetworkTable m_shooterTable = NetworkTableInstance.getDefault().getTable("limelight-shooter"); // Makes the Limelight data table.
  private static NetworkTableEntry m_botPose_shooter = m_shooterTable.getEntry("botpose_wpiblue"); // Makes a double array to hold the robot pose.
  private static NetworkTableEntry m_tv_shooter = m_shooterTable.getEntry("tv");
  private static NetworkTableEntry m_tx_shooter = m_shooterTable.getEntry("tx");
  private static NetworkTableEntry m_ty_shooter = m_shooterTable.getEntry("ty");
  private static NetworkTableEntry m_ta_shooter = m_shooterTable.getEntry("ta");
  private static NetworkTableEntry m_tl_shooter = m_shooterTable.getEntry("tl");
  private static NetworkTableEntry m_cl_shooter = m_shooterTable.getEntry("cl");

  private static final NetworkTable m_collectTable = NetworkTableInstance.getDefault().getTable("limelight-collect"); // Makes the Limelight data table.
  private static NetworkTableEntry m_botPose_collect = m_collectTable.getEntry("botpose_wpiblue"); // Makes a double array to hold the robot pose.
  private static NetworkTableEntry m_tv_collect = m_collectTable.getEntry("tv");
  private static NetworkTableEntry m_tx_collect = m_collectTable.getEntry("tx");
  private static NetworkTableEntry m_ty_collect = m_collectTable.getEntry("ty");
  private static NetworkTableEntry m_ta_collect = m_collectTable.getEntry("ta");
  private static NetworkTableEntry m_tl_collect = m_collectTable.getEntry("tl");
  private static NetworkTableEntry m_cl_collect = m_collectTable.getEntry("cl");

 // private NetworkTableEntry m_

  /** Creates a new Limelight. */
  public Limelight() {}

  public static double getLatency(LimelightConstants.Device device) {
    if (device == LimelightConstants.Device.SHOOTER ) {
      return m_tl_shooter.getDouble(0.0); //Returns latency
    } else {
      return m_tl_collect.getDouble(0.0); //Returns latency
    }
  }

  public static double getTotalLatency(LimelightConstants.Device device) {
    if (device == LimelightConstants.Device.SHOOTER ) {
      return m_cl_shooter.getDouble(0.0); //Returns total latency
    } else {
      return m_cl_collect.getDouble(0.0); //Returns total latency
    }
   }

   public static double getTargetArea(LimelightConstants.Device device){
    if (device == LimelightConstants.Device.SHOOTER ) {
      return m_ta_shooter.getDouble(0.0); //Returns target area
    } else {
      return m_ta_collect.getDouble(0.0); //Returns target area
    }
  }

  public static double getHorizontalOffset(LimelightConstants.Device device) {
    if (device == LimelightConstants.Device.SHOOTER ) {
      return m_tx_shooter.getDouble(0.0); // Returns the horizontal offset from valid target
    } else {
      return m_tx_collect.getDouble(0.0); // Returns the horizontal offset from valid target
    }
  }

  public static double getVerticalOffset(LimelightConstants.Device device) {
    if (device == LimelightConstants.Device.SHOOTER ) {
      return m_ty_shooter.getDouble(0.0); // Returns the vertical offset from valid target
    } else {
      return m_ty_collect.getDouble(0.0); // Returns the vertical offset from valid target
    }
  }

  public static double getDistance(LimelightConstants.Device device) {
    if (device == LimelightConstants.Device.SHOOTER ) {
      return m_botPose_shooter.getDouble(0.0); // TODO: Returns the current distance.
    } else {
      return m_botPose_collect.getDouble(0.0); // TODO: Returns the current distance.
    }
  }

  public static boolean isValidTargetSeen (LimelightConstants.Device device) {
    double result = 0.0;
    if (device == LimelightConstants.Device.SHOOTER ) {
       result = m_tv_shooter.getDouble(0.0); //Returns if a valid target is seen or not
    } else {
       result = m_tv_collect.getDouble(0.0); //Returns if a valid target is seen or not
    }

    return result == 1.0;
  }

  public static double[] getBotPose(LimelightConstants.Device device) {
    if (device == LimelightConstants.Device.SHOOTER ) {
      return  m_botPose_shooter.getDoubleArray(new double[7]); //Returns field space robot pose
    } else {
      return  m_botPose_collect.getDoubleArray(new double[7]); //Returns field space robot pose
    }
  }

  private static int countStringOccurances(String str, String subStr){
    int occurances = 0;
    for (int i =0; i< str.length()-subStr.length()+1; i++){
      if(str.substring(i, i+subStr.length()).equals(subStr)){
        occurances++;
      }
    }
    return occurances;
    }

  //Deprecated : JSON is disabled by default, and we can now get the number of visable apriltags from the botpose buffer
  public static int getNumTargets(LimelightConstants.Device device){
    int retVal = 0;
    /* 
    int retVal = 0;
    if (device == LimelightConstants.Device.SHOOTER ) {
      retVal = countStringOccurances(m_shooterTable.getEntry("json").getString(""), "pts");
    } else {
      retVal = countStringOccurances(m_collectTable.getEntry("json").getString(""), "pts");
    }
    //System.out.println("Num targets seen = "+ retVal);*/

    double [] poseBuf = Limelight.getBotPose(LimelightConstants.Device.SHOOTER);

    if (poseBuf.length >= 7){
      retVal = (int) poseBuf[7];
    }

    


    return retVal;
  }

}






