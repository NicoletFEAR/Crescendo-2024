// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.utilities.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public static final boolean kInfoMode = true;

  // // DRIVEBASE \\ \\

  // Make sure to measure these with as much presicion as possible, as it will have great affect on
  // path planner autos and teleop driving

  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class AutoConstants {
    public static Translation2d[] kNotePlacements = new Translation2d[] {
      new Translation2d(7.68, 7.44),
      new Translation2d(7.68, 5.78),
      new Translation2d(7.68, 4.10),
      new Translation2d(7.68, 2.44),
      new Translation2d(7.68, 0.78)
    };
  }

  public static final class DriveConstants {

    public static double drivekp = 0.15751;
    public static double driveki = 0.0;
    public static double drivekd = 0.0;
    public static double drivekff = 0.23983;
    public static double driverampRate = 0.1;

    public static double turnkp = 0.02;
    public static double turnki = 0.0;
    public static double turnkd = 0.01;
    public static double turnkff = 0.0;

    public static double kDriveModifier = 2;
    public static double kTurnModifier = 3;

    /* The lower this is the more you want odometry to trust the april tags
    Scales based on the percentage of an april tag in view
    Dont do anything below 0*/
    public static double kAprilTagTrustMultiplier = 1.0;

    public static int kPigeon = 47;

    public static double kSteerVelocityDeadzone = 0.005;

    public static final double kTrackWidth =
        Units.inchesToMeters(20.67); // Distance between centers of right and left wheels on robot

    public static final double kWheelBase =
        Units.inchesToMeters(20.67); // Distance between centers of front and back wheels on robot

    public static final double kDriveBaseRadius =
        Math.hypot(
            Units.inchesToMeters(32) / 2,
            Units.inchesToMeters(32)
                / 2); // Distance from center of the robot to corner of the bumpers

    public static final double kMaxMetersPerSecond =
        Units.feetToMeters(20.1); // Run drivebase at max speed on the ground to find top speed

    public static final double kDriveGearRatio = 5.90318; // MK4i L2 with 16t driving gear, find on sds website

    public static final double kTurnGearRatio =
        150.0 / 7.0; // MK4i turning ratio MK4i Neo, find on sds website

    public static final double kWheelDiameter = Units.inchesToMeters(3.915); // Wheel diameter

    public static final double kMaxRotationRadiansPerSecond =
        Math.PI * 3.0; // Just kind of find what works, this is from 930 2023

    public static final double kRegularSpeed = 1; // Regular speed multiplier of robot

    public static final double kSlowSpeed = .4; // Slow speed multiplier of robot

    public static final int kFrontLeftDriveMotor = 12;
    public static final int kFrontLeftSteerMotor = 11;
    public static final int kFrontLeftSteerEncoder = 1;
    public static final double kFrontLeftOffset = 0.617676; //0.637451 ; // 0.719971; //0.597900; // In Rotations not degrees
    public static final SwerveModuleConstants kFrontLeft =
        new SwerveModuleConstants(
            kFrontLeftDriveMotor, kFrontLeftSteerMotor, kFrontLeftSteerEncoder, kFrontLeftOffset);

    public static final int kFrontRightDriveMotor = 18;
    public static final int kFrontRightSteerMotor = 17;
    public static final int kFrontRightSteerEncoder = 7;
    public static final double kFrontRightSteerOffset = 0.046875; // 0.052979; //0.049561; // In Rotations not degrees
    public static final SwerveModuleConstants kFrontRight =
        new SwerveModuleConstants(
            kFrontRightDriveMotor,
            kFrontRightSteerMotor,
            kFrontRightSteerEncoder,
            kFrontRightSteerOffset);

    public static final int kBackLeftDriveMotor = 14;
    public static final int kBackLeftSteerMotor = 13;
    public static final int kBackLeftSteerEncoder = 3;
    public static final double kBackLeftSteerOffset = 0.503662; //0.502930  ; //0.504395 ; // In Rotations not degrees
    public static final SwerveModuleConstants kBackLeft =
        new SwerveModuleConstants(
            kBackLeftDriveMotor, kBackLeftSteerMotor, kBackLeftSteerEncoder, kBackLeftSteerOffset);

    public static final int kBackRightDriveMotor = 16;
    public static final int kBackRightSteerMotor = 15;
    public static final int kBackRightSteerEncoder = 5;
    public static final double kBackRightSteerOffset = 0.831055;//0.832520 ; //0.829590 ;//0.834473; // In Rotations not degrees
    public static final SwerveModuleConstants kBackRight =
        new SwerveModuleConstants(
            kBackRightDriveMotor,
            kBackRightSteerMotor,
            kBackRightSteerEncoder,
            kBackRightSteerOffset);

    public static final double kDriveRevToMeters = ((kWheelDiameter * Math.PI) / kDriveGearRatio);
    public static final double kDriveRpmToMetersPerSecond = kDriveRevToMeters / 60.0;
    public static final double kTurnRotationsToDegrees = 360.0 / kTurnGearRatio;

    public static final Translation2d[] kModuleTranslations = {
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

    public static final PIDConstants kPathPlannerTranslationPID = new PIDConstants(5.0, 0, 0);
    public static final PIDConstants kPathPlannerRotationPID = new PIDConstants(5.0, 0, 0);

    public static final SwerveModuleState[] kXWheels = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-135)),
    };

    public static final Translation2d kBlueSpeakerPosition = new Translation2d(0, 5.56);
    public static final Translation2d kRedSpeakerPosition = new Translation2d(16.53, 5.56);
    public static final Translation2d kBlueAmpPassPosition = new Translation2d(1.32, 7.29);
    public static final Translation2d kRedAmpPassPosition = new Translation2d(14.72, 5.56);
    public static final Translation2d kBlueAmpPosition = new Translation2d(1.82, 7.69);
    public static final Translation2d kRedAmpPosition = new Translation2d(14.71, 7.69);
  }

  public class MotorConstants {
    // LAUNCHER \\
    public static final int kLauncherTopFlywheelID = 43;
    public static final int kLauncherBottomFlywheelID = 41;
    public static final int kLauncherWristID = 40;
    public static final int kLauncherHoldID = 44;

    // INTAKE \\
    public static final int kElevatorLiftRightID = 20;
    public static final int kElevatorLiftLeftID = 21;
    
    public static final int kIntakeFlywheelID = 34;
    public static final int kIntakeWristID = 31;
    public static final int kIntakeHoldID = 36;
    // CLIMB \\
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
