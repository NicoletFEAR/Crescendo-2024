// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utilities.GeometryUtils;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.swerve.SwerveDrive;

public class Limelight extends SubsystemBase {

  private static Limelight m_launchInstance = null;
  private static Limelight m_intakeInstance = null;

  private NetworkTableInstance m_visionTable;
  private NetworkTableEntry m_tv;
  private NetworkTableEntry m_ta;
  private NetworkTableEntry m_tl;
  private NetworkTableEntry m_cl;
  private NetworkTableEntry m_tcornxy;
  private NetworkTableEntry m_botpose;
  private NetworkTableEntry m_targetpose_robotspace;

  NetworkTable currentData;

  public double area;
  public int fidId;
  public String camName;

  public static Limelight getLaunchLimelight() {
    if (m_launchInstance == null) {
      m_launchInstance = new Limelight("limelight-launch");
    }

    return m_launchInstance;
  }

  public static Limelight getIntakeLimelight() {
    if (m_intakeInstance == null) {
      m_intakeInstance = new Limelight("limelight-intake");
    }

    return m_intakeInstance;
  }

  public Limelight(String name) {
    m_visionTable = NetworkTableInstance.getDefault();
    m_tv = m_visionTable.getEntry("tv");
    m_ta = m_visionTable.getEntry("ta");
    m_tl = m_visionTable.getEntry("tl");
    m_cl = m_visionTable.getEntry("cl");
    m_tcornxy = m_visionTable.getEntry("tcornxy");
    m_botpose = m_visionTable.getEntry("botpose");
    m_targetpose_robotspace = m_visionTable.getEntry("targetpose_robotspace");

    camName = name;
  }

  public double getA() {
    return m_ta.getDouble(0.0);
  }

  public double getV() {
    return m_tv.getDouble(0.0);
  }

  public double[] getBotPose() {
    return m_botpose.getDoubleArray(new double[7]);
  }

  public Pose2d getLimelightPose() {
    double[] botpose = getBotPose();

    return new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
  }

  public Pose3d getTargetPose() {
    double[] targetpose = m_targetpose_robotspace.getDoubleArray(new double[7]);

    return new Pose3d(
        targetpose[0],
        targetpose[1],
        targetpose[2],
        new Rotation3d(targetpose[3], targetpose[4], targetpose[5]));
  }

  @Override
  public void periodic() {
    currentData = m_visionTable.getTable(camName);
    m_tv = currentData.getEntry("tv");
    m_ta = currentData.getEntry("ta");
    m_tl = currentData.getEntry("tl");
    m_cl = currentData.getEntry("cl");
    m_tcornxy = currentData.getEntry("tcornxy");
    m_botpose = currentData.getEntry("botpose_wpiblue");
    m_targetpose_robotspace = currentData.getEntry("targetpose_robotspace");


  }
}
