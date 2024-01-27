// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.TreeMap;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.FalconTestingStateMachine.FalconTestingState;
import frc.robot.subsystems.launcher.LauncherFlywheel.LauncherFlywheelState;
import frc.robot.subsystems.launcher.LauncherWrist.LauncherWristState;
import frc.robot.subsystems.templates.VelocitySubsystem.VelocitySubsystemType;
import frc.robot.subsystems.templates.PositionSubsystem.PositionSubsystemType;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.SparkMaxConstants;
import frc.robot.subsystems.templates.SubsystemConstants.TalonFXConstants;
import frc.robot.subsystems.templates.SubsystemConstants.TalonFXPositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;

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

  public static final boolean kInfoMode = true;

  public static final boolean kTuningMode = false;

  // // DRIVEBASE \\ \\

  // Make sure to measure these with as much presicion as possible, as it will have great affect on
  // path planner autos and teleop driving

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;
  }

  public static final class FalconTestingConstants {
    public static final TalonFXConstants kFalconTestingMasterConstants = new TalonFXConstants();
    static {
      kFalconTestingMasterConstants.kID = 6;
      kFalconTestingMasterConstants.kGravityType = GravityTypeValue.Elevator_Static;
      kFalconTestingMasterConstants.kKp = 0.1;
      kFalconTestingMasterConstants.kKi = 0.0;
      kFalconTestingMasterConstants.kKd = 0.0;
      kFalconTestingMasterConstants.kKs = 0.0;
      kFalconTestingMasterConstants.kKg = 0.0;
      kFalconTestingMasterConstants.kKv = 0.0;
      kFalconTestingMasterConstants.kKa = 0.0;
      kFalconTestingMasterConstants.kMaxAcceleration = 5.0;
      kFalconTestingMasterConstants.kMaxVelocity = 10.0;
      kFalconTestingMasterConstants.kMaxJerk = 1.0;
      kFalconTestingMasterConstants.kNuetralMode = NeutralModeValue.Brake;
      kFalconTestingMasterConstants.kIsInverted = InvertedValue.Clockwise_Positive;
    }

    public static final TalonFXPositionSubsystemConstants kFalconTestingConstants = new TalonFXPositionSubsystemConstants(); 
    static {
      kFalconTestingConstants.kName = "Falcon Testing";
      kFalconTestingConstants.kSubsystemType = PositionSubsystemType.FALCON_TESTING;
      kFalconTestingConstants.kMasterConstants = kFalconTestingMasterConstants;
      kFalconTestingConstants.kSlaveConstants = new TalonFXConstants[0];
      kFalconTestingConstants.kInitialState = FalconTestingState.DOWN;
      kFalconTestingConstants.kManualState = FalconTestingState.MANUAL;
      kFalconTestingConstants.kTransitionState = FalconTestingState.TRANSITION;
      kFalconTestingConstants.kDefaultSlot = 0; // PID Slot, make more if more than one set of pid constants are used
      kFalconTestingConstants.kHomePosition = 0.0;
      kFalconTestingConstants.kPositionConversionFactor = 1.0; // To find degrees: 360/gear ration ex 360/100 for 100:1
      kFalconTestingConstants.kSetpointTolerance = 0.0; // Tolerance for atSetpoint()
      kFalconTestingConstants.kMaxPosition = Double.POSITIVE_INFINITY;
      kFalconTestingConstants.kMinPosition = Double.NEGATIVE_INFINITY;
      kFalconTestingConstants.kManualControlMode = ManualControlMode.BUMPERS;
      kFalconTestingConstants.kManualMultiplier = 1;
      kFalconTestingConstants.kManualDeadBand = .1;
    } 
  }

  public static final class LauncherConstants {

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

    public static final SparkMaxConstants kLauncherFlywheelMasterConstants = new SparkMaxConstants();

    static {
      kLauncherFlywheelMasterConstants.kID = 34;
      kLauncherFlywheelMasterConstants.kIdleMode = IdleMode.kBrake;
      kLauncherFlywheelMasterConstants.kMotorType = MotorType.kBrushless;
      kLauncherFlywheelMasterConstants.kCurrentLimit = 80;
      kLauncherFlywheelMasterConstants.kInverted = false;
    }

    public static final SparkMaxConstants[] kLauncherFlywheelSlaveConstants = new SparkMaxConstants[0];

    public static final VelocitySubsystemConstants kLauncherFlywheelConstants =
        new VelocitySubsystemConstants();

    static {
      kLauncherFlywheelConstants.kName = "Launcher Flywheel";

      kLauncherFlywheelConstants.kSubsystemType = VelocitySubsystemType.LAUNCHER_FLYWHEEL;

      kLauncherFlywheelConstants.kMasterConstants = kLauncherFlywheelMasterConstants;
      kLauncherFlywheelConstants.kSlaveConstants = kLauncherFlywheelSlaveConstants;

      kLauncherFlywheelConstants.kVelocityConversionFactor = 3 / 60; // division by 60 to get rotations per second

      kLauncherFlywheelConstants.kKp = 0.01;
      kLauncherFlywheelConstants.kKi = 0.0;
      kLauncherFlywheelConstants.kKd = 0.0;

      kLauncherFlywheelConstants.kDefaultSlot = 0;

      kLauncherFlywheelConstants.kKs = 0.0;
      kLauncherFlywheelConstants.kKv = 0.0;
      kLauncherFlywheelConstants.kKa = 0.0;

      kLauncherFlywheelConstants.kInitialState = LauncherFlywheelState.OFF;
      kLauncherFlywheelConstants.kTransitionState = LauncherFlywheelState.TRANSITION;
    }

  public static final SparkMaxConstants kLauncherWristMasterConstants = new SparkMaxConstants();

    static {
      kLauncherWristMasterConstants.kID = 7;
      kLauncherWristMasterConstants.kIdleMode = IdleMode.kBrake;
      kLauncherWristMasterConstants.kMotorType = MotorType.kBrushless;
      kLauncherWristMasterConstants.kCurrentLimit = 80;
      kLauncherWristMasterConstants.kInverted = false;
    }

    public static final SparkMaxConstants[] kWristSlaveConstants = new SparkMaxConstants[0];

    public static final PositionSubsystemConstants kLauncherWristConstants =
        new PositionSubsystemConstants();

    static {
      kLauncherWristConstants.kName = "Launcher Wrist";

      kLauncherWristConstants.kSubsystemType = PositionSubsystemType.LAUNCHER_WRIST;

      kLauncherWristConstants.kMasterConstants = kLauncherWristMasterConstants;
      kLauncherWristConstants.kSlaveConstants = kWristSlaveConstants;

      kLauncherWristConstants.kHomePosition = 155;
      kLauncherWristConstants.kPositionConversionFactor = 360 / 100;

      kLauncherWristConstants.kKp = 0.2;
      kLauncherWristConstants.kKi = 0.0;
      kLauncherWristConstants.kKd = 0.0;
      kLauncherWristConstants.kSetpointTolerance = 0.1;
      kLauncherWristConstants.kSmartMotionTolerance = 0.1;

      kLauncherWristConstants.kDefaultSlot = 0;

      kLauncherWristConstants.kMaxVelocity = 2000;
      kLauncherWristConstants.kMaxAcceleration = 2000;

      kLauncherWristConstants.kKs = 0.0;
      kLauncherWristConstants.kKg = 0.0;
      kLauncherWristConstants.kKv = 0.0;
      kLauncherWristConstants.kKa = 0.0;

      kLauncherWristConstants.kMaxPosition = 155;
      kLauncherWristConstants.kMinPosition = -85;

      kLauncherWristConstants.kManualControlMode = ManualControlMode.TRIGGERS;
      kLauncherWristConstants.kManualMultiplier = 1;
      kLauncherWristConstants.kManualDeadBand = .1;

      kLauncherWristConstants.kInitialState = LauncherWristState.DOWN;
      kLauncherWristConstants.kManualState = LauncherWristState.MANUAL;
      kLauncherWristConstants.kTransitionState = LauncherWristState.TRANSITION;
    }
  }


  public static enum ManualControlMode {
    TRIGGERS,
    BUMPERS,
    LEFT_X,
    LEFT_Y,
    RIGHT_X,
    RIGHT_Y
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
