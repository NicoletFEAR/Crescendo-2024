// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.TreeMap;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
import frc.robot.subsystems.launcher.LauncherWrist.LauncherWristState;
import frc.robot.subsystems.templates.VelocitySubsystem.VelocitySubsystemType;
import frc.robot.subsystems.templates.VoltageSubsystem.VoltageSubsystemType;
import frc.robot.subsystems.templates.PositionSubsystem.PositionSubsystemType;
import frc.robot.subsystems.templates.SubsystemConstants.ManualControlMode;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;
import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.VoltageSubsystemConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public static final boolean kTuningMode = true;

  // // DRIVEBASE \\ \\

  // Make sure to measure these with as much presicion as possible, as it will have great affect on
  // path planner autos and teleop driving

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;
  }


  public static final class LauncherConstants {

    
    public static final int kWristCANCoderId = 4;
    public static final int kLaunchBeamBreakId = 0;


    // IN METERS
    public static final TreeMap<Double, Double> kDistanceRPMMap = new TreeMap<>();
    static {
      kDistanceRPMMap.put(0.0, 1000.0);
      kDistanceRPMMap.put(1.0, 2000.0);
      kDistanceRPMMap.put(1.5, 2500.0);
      kDistanceRPMMap.put(2.0, 3000.0);
      kDistanceRPMMap.put(2.5, 3500.0);
      kDistanceRPMMap.put(3.0, 4000.0);
      kDistanceRPMMap.put(3.5, 4500.0);
      kDistanceRPMMap.put(4.0, 5000.0);
      kDistanceRPMMap.put(4.5, 5500.0);
      kDistanceRPMMap.put(5.0, 6000.0);
      kDistanceRPMMap.put(5.5, 6500.0);
    }

    // IN DEGREES
    public static final TreeMap<Double, Double> kDistancePitchMap = new TreeMap<>();
    static {
      kDistancePitchMap.put(0.0, 90.0);
      kDistancePitchMap.put(1.0, 60.0);
      kDistancePitchMap.put(1.5, 55.0);
      kDistancePitchMap.put(2.0, 50.0);
      kDistancePitchMap.put(2.5, 45.0);
      kDistancePitchMap.put(3.0, 40.0);
      kDistancePitchMap.put(3.5, 35.0);
      kDistancePitchMap.put(4.0, 30.0);
      kDistancePitchMap.put(4.5, 25.0);
      kDistancePitchMap.put(5.0, 20.0);
      kDistancePitchMap.put(5.5, 15.0);
      kDistancePitchMap.put(6.0, 10.0);
    }

    public static final SparkConstants kTopLauncherFlywheelConstants = new SparkConstants();

    static {
      kTopLauncherFlywheelConstants.kID = 16;
      kTopLauncherFlywheelConstants.kRevMotorType = RevMotorType.CAN_SPARK_FLEX;
      kTopLauncherFlywheelConstants.kName = "Top Launcher Flywheel";
      kTopLauncherFlywheelConstants.kIdleMode = IdleMode.kCoast;
      kTopLauncherFlywheelConstants.kMotorType = MotorType.kBrushless;
      kTopLauncherFlywheelConstants.kCurrentLimit = 80;
      kTopLauncherFlywheelConstants.kInverted = false;
      kTopLauncherFlywheelConstants.kKp = 0.00001;
      kTopLauncherFlywheelConstants.kKi = 0.0;
      kTopLauncherFlywheelConstants.kKd = 0.0;
      kTopLauncherFlywheelConstants.kKff = 0.0001675;
    }

    public static final SparkConstants kBottomLauncherFlywheelConstants = new SparkConstants();

    static {
      kBottomLauncherFlywheelConstants.kID = 18;
      kBottomLauncherFlywheelConstants.kRevMotorType = RevMotorType.CAN_SPARK_FLEX;
      kBottomLauncherFlywheelConstants.kName = "Bottom Launcher Flywheel";
      kBottomLauncherFlywheelConstants.kIdleMode = IdleMode.kCoast;
      kBottomLauncherFlywheelConstants.kMotorType = MotorType.kBrushless;
      kBottomLauncherFlywheelConstants.kCurrentLimit = 80;
      kBottomLauncherFlywheelConstants.kInverted = false;
      kBottomLauncherFlywheelConstants.kKp = 0.00001;
      kBottomLauncherFlywheelConstants.kKi = 0.0;
      kBottomLauncherFlywheelConstants.kKd = 0.0;
      kBottomLauncherFlywheelConstants.kKff = 0.0001675;
    }

    public static final VelocitySubsystemConstants kLauncherFlywheelConstants =
        new VelocitySubsystemConstants();

    static {
      kLauncherFlywheelConstants.kSubsystemName = "Launcher Flywheel";
      kLauncherFlywheelConstants.kSuperstructureName = "Launcher";

      kLauncherFlywheelConstants.kSubsystemType = VelocitySubsystemType.LAUNCHER_FLYWHEEL;

      kLauncherFlywheelConstants.kMotorConstants = new SparkConstants[] {kTopLauncherFlywheelConstants, kBottomLauncherFlywheelConstants};

      kLauncherFlywheelConstants.kVelocityConversionFactor = 1.0; 

      kLauncherFlywheelConstants.kDefaultSlot = 0;

      kLauncherFlywheelConstants.kSetpointTolerance = 1.0;

      kLauncherFlywheelConstants.kInitialState = LauncherFlywheelState.OFF;
      kLauncherFlywheelConstants.kTransitionState = LauncherFlywheelState.TRANSITION;
      kLauncherFlywheelConstants.kManualState = LauncherFlywheelState.MANUAL;
    }

  public static final SparkConstants kLauncherWristLeaderConstants = new SparkConstants();

    static {
      kLauncherWristLeaderConstants.kID = 34;
      kLauncherWristLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
      kLauncherWristLeaderConstants.kIdleMode = IdleMode.kBrake;
      kLauncherWristLeaderConstants.kMotorType = MotorType.kBrushless;
      kLauncherWristLeaderConstants.kCurrentLimit = 80;
      kLauncherWristLeaderConstants.kInverted = false;
      kLauncherWristLeaderConstants.kKp = 0.1;
      kLauncherWristLeaderConstants.kKi = 0.0;
      kLauncherWristLeaderConstants.kKd = 0.0;
      kLauncherWristLeaderConstants.kKff = 0.0;
    }

    public static final SparkConstants[] kWristSlaveConstants = new SparkConstants[0];

    public static final PositionSubsystemConstants kLauncherWristConstants =
        new PositionSubsystemConstants();

    static {
      kLauncherWristConstants.kSubsystemName = "Launcher Wrist";
      kLauncherWristConstants.kSuperstructureName = "Launcher";

      kLauncherWristConstants.kSubsystemType = PositionSubsystemType.LAUNCHER_WRIST;

      kLauncherWristConstants.kLeaderConstants = kLauncherWristLeaderConstants;
      kLauncherWristConstants.kFollowerConstants = kWristSlaveConstants;

      kLauncherWristConstants.kHomePosition = 155;
      kLauncherWristConstants.kPositionConversionFactor = 360;

      kLauncherWristConstants.kSetpointTolerance = 0.1;

      kLauncherWristConstants.kDefaultSlot = 0;

      kLauncherWristConstants.kMaxVelocity = 100;
      kLauncherWristConstants.kMaxAcceleration = 50;

      kLauncherWristConstants.kMaxPosition = 155;
      kLauncherWristConstants.kMinPosition = -85;

      kLauncherWristConstants.kManualControlMode = ManualControlMode.TRIGGERS;
      kLauncherWristConstants.kManualMultiplier = 1;
      kLauncherWristConstants.kManualDeadBand = .1;

      kLauncherWristConstants.kInitialState = LauncherWristState.DOWN;
      kLauncherWristConstants.kManualState = LauncherWristState.MANUAL;
      kLauncherWristConstants.kTransitionState = LauncherWristState.TRANSITION;
    }

    public static final SparkConstants kLauncherHoldLeaderConstants = new SparkConstants();

    static {
      kLauncherHoldLeaderConstants.kID = 36;
      kLauncherHoldLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
      kLauncherHoldLeaderConstants.kIdleMode = IdleMode.kBrake;
      kLauncherHoldLeaderConstants.kMotorType = MotorType.kBrushless;
      kLauncherHoldLeaderConstants.kCurrentLimit = 80;
      kLauncherHoldLeaderConstants.kInverted = false;
    }

    public static final VoltageSubsystemConstants kLauncherHoldConstants = new VoltageSubsystemConstants();

    static {
      kLauncherHoldConstants.kSubsystemName = "Launcher Hold";
      kLauncherHoldConstants.kSuperstructureName = "Launcher";

      kLauncherHoldConstants.kSubsystemType = VoltageSubsystemType.LAUNCHER_HOLD;

      kLauncherHoldConstants.kLeaderConstants = kLauncherHoldLeaderConstants;
      kLauncherHoldConstants.kFollowerConstants = new SparkConstants[0];

      kLauncherHoldConstants.kInitialState = null;
    }

  }



  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
