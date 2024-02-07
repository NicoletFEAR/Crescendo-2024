// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.intake.IntakeFlywheel.IntakeFlywheelState;
import frc.robot.subsystems.templates.PositionSubsystem.PositionSubsystemType;
import frc.robot.subsystems.templates.SubsystemConstants.ManualControlMode;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;
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

  public static final boolean kInfoMode = false;

  public static final boolean kTuningMode = false;

  // // DRIVEBASE \\ \\

  // Make sure to measure these with as much presicion as possible, as it will have great affect on
  // path planner autos and teleop driving

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;
  }
  public static final class IntakeConstants {

    public static final SparkConstants kIntakeFlywheelMasterConstants = new SparkConstants();

    static {
      kIntakeFlywheelMasterConstants.kID = 30;
      kIntakeFlywheelMasterConstants.kIdleMode = IdleMode.kBrake;
      kIntakeFlywheelMasterConstants.kMotorType = MotorType.kBrushless;
      kIntakeFlywheelMasterConstants.kCurrentLimit = 80;
      kIntakeFlywheelMasterConstants.kInverted = false;
    }

    // public static final SparkConstants[] kIntakeFlywheelSlaveConstants = new SparkConstants[1];
    // static {
    //   kIntakeFlywheelSlaveConstants[0] = new SparkConstants();
    //   kIntakeFlywheelSlaveConstants[0].kID = 18;
    //   kIntakeFlywheelSlaveConstants[0].kIdleMode = IdleMode.kBrake;
    //   kIntakeFlywheelSlaveConstants[0].kMotorType = MotorType.kBrushless;
    //   kIntakeFlywheelSlaveConstants[0].kCurrentLimit = 80;
    //   kIntakeFlywheelSlaveConstants[0].kInverted = true;
    // }

    public static final VoltageSubsystemConstants kIntakeFlywheelConstants =
        new VoltageSubsystemConstants();

    static {
      kIntakeFlywheelConstants.kSubsystemName = "Intake Flywheel";

      // kIntakeFlywheelConstants.kSubsystemType = VoltageSubsystemType.INTAKE_FLYWHEEL;

      kIntakeFlywheelConstants.kLeaderConstants = kIntakeFlywheelMasterConstants;
      //kIntakeFlywheelConstants.kFollowerConstants = kIntakeFlywheelSlaveConstants;

      kIntakeFlywheelConstants.kInitialState = IntakeFlywheelState.OFF;
      // kIntakeFlywheelConstants.kTransitionState = IntakeFlywheelState.TRANSITION;
    }

  public static final SparkConstants kIntakeWristMasterConstants = new SparkConstants();

    static {
      kIntakeWristMasterConstants.kID = 34;
      kIntakeWristMasterConstants.kIdleMode = IdleMode.kBrake;
      kIntakeWristMasterConstants.kMotorType = MotorType.kBrushless;
      kIntakeWristMasterConstants.kCurrentLimit = 80;
      kIntakeWristMasterConstants.kInverted = false;
    }

    public static final SparkConstants[] kWristSlaveConstants = new SparkConstants[0];

    public static final PositionSubsystemConstants kIntakeWristConstants =
        new PositionSubsystemConstants();

    static {
      kIntakeWristConstants.kSubsystemName = "Launcher Wrist";

      kIntakeWristConstants.kSubsystemType = PositionSubsystemType.LAUNCHER_WRIST;

      kIntakeWristConstants.kLeaderConstants = kIntakeWristMasterConstants;
      kIntakeWristConstants.kFollowerConstants = kWristSlaveConstants;

      kIntakeWristConstants.kHomePosition = 155;
      kIntakeWristConstants.kPositionConversionFactor = 360 / 100;

      kIntakeWristConstants.kDefaultSlot = 0;

      kIntakeWristConstants.kMaxVelocity = 100;
      kIntakeWristConstants.kMaxAcceleration = 50;

      kIntakeWristConstants.kMaxPosition = 155;
      kIntakeWristConstants.kMinPosition = -85;

      kIntakeWristConstants.kManualControlMode = ManualControlMode.TRIGGERS;
      kIntakeWristConstants.kManualMultiplier = 1;
      kIntakeWristConstants.kManualDeadBand = .1;

      // kIntakeWristConstants.kInitialState = LauncherWristState.DOWN;
      // kIntakeWristConstants.kManualState = LauncherWristState.MANUAL;
      // kIntakeWristConstants.kTransitionState = LauncherWristState.TRANSITION;
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
