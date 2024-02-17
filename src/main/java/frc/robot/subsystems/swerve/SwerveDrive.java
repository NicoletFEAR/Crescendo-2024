// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.lib.utilities.GeometryUtils;
import frc.lib.utilities.SwerveModuleConstants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SwerveDrive extends SubsystemBase {

  private static SwerveDrive m_instance = null;

  private final Pigeon2 m_pigeon = new Pigeon2(Constants.DriveConstants.kPigeon, "CANivore");

  private Pose2d targetPPPose = new Pose2d(0, 0, new Rotation2d(0));
  public static List<Pose2d> ppPath = new ArrayList<>();

  public final Field2d m_field = new Field2d();

  private PIDController m_snapToAngleController;

  private AngleToSnap m_angleToSnap = AngleToSnap.NONE;

  private boolean m_xWheels = false;

  private double m_simyaw = 0;

  private static final boolean invertGyro = false;

  private SwerveModuleState[] moduleStates;

  private SwerveModule[] m_SwerveMods;

  private ChassisSpeeds robotRelativeChassisSpeeds;

  private SwerveDrivePoseEstimator poseEstimator;

  /**
   *
   *
   * <h3>SwerveDrive</h3>
   *
   * Builds the swerve drivebase from the swerve module class
   *
   * @param frontLeftModuleConstants
   * @param frontRightModuleConstants
   * @param backLeftModuleConstants
   * @param backRightModuleConstants
   */
  public SwerveDrive(
      SwerveModuleConstants frontLeftModuleConstants,
      SwerveModuleConstants frontRightModuleConstants,
      SwerveModuleConstants backLeftModuleConstants,
      SwerveModuleConstants backRightModuleConstants) {

    m_SwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, frontLeftModuleConstants),
          new SwerveModule(1, frontRightModuleConstants),
          new SwerveModule(2, backLeftModuleConstants),
          new SwerveModule(3, backRightModuleConstants)
        };

    m_pigeon.optimizeBusUtilization();
    m_pigeon.getYaw().setUpdateFrequency(50);
    m_pigeon.getFault_BootupGyroscope().setUpdateFrequency(4);

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetAngleToAbsolute();

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.9, 0.9, 0.9));

    m_pigeon.setYaw(0);

    robotRelativeChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    m_snapToAngleController = new PIDController(.06, 0, 0);

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          targetPPPose = targetPose;
        });

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          ppPath = activePath;
        });

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::updateEstimatorWithPose, // Method to reset odometry (will be called if your auto has
        // a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            DriveConstants.kPathPlannerTranslationPID, // Translation PID constants
            DriveConstants.kPathPlannerRotationPID, // Rotation PID constants
            DriveConstants.kMaxMetersPerSecond, // Max module speed, in m/s
            DriveConstants
                .kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
          },
        this // Reference to this subsystem to set requirements
        );
  }

  public static SwerveDrive getInstance() {
    if (m_instance == null) {
      m_instance =
          new SwerveDrive(
              DriveConstants.kFrontLeft,
              DriveConstants.kFrontRight,
              DriveConstants.kBackLeft,
              DriveConstants.kBackRight);
    }

    return m_instance;
  }

  public boolean shouldFlip() {
    return DriverStation.getAlliance().get() == Alliance.Blue ? false : true;
  }

  public void zeroGyroscope() {
    m_pigeon.setYaw(0);
  }

  public void drive(
      double throttle, double strafe, double rotation, boolean isOpenLoop, boolean fieldRelative) {

    if (m_angleToSnap != AngleToSnap.NONE) {
      if (Math.abs(m_angleToSnap.getAngle() - getYawDegrees()) < 1) {
        m_angleToSnap = AngleToSnap.NONE;
      } else {
        rotation =
            MathUtil.clamp(
                m_snapToAngleController.calculate(
                    GeometryUtils.getAdjustedYawDegrees(getYawDegrees(), m_angleToSnap.getAngle()),
                    180),
                -1,
                1);
      }
    }

    if (throttle + strafe + rotation != 0 && m_xWheels == true) {
      m_xWheels = false;
    }

    if (m_xWheels == false) {
      throttle = throttle * DriveConstants.kMaxMetersPerSecond;
      strafe = strafe * DriveConstants.kMaxMetersPerSecond;
      rotation = rotation * DriveConstants.kMaxRotationRadiansPerSecond;

      ChassisSpeeds chassisSpeeds =
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, getYaw())
              : new ChassisSpeeds(throttle, strafe, rotation);

      chassisSpeeds = GeometryUtils.discretize(chassisSpeeds);
      moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      setSwerveModuleStates(moduleStates, isOpenLoop);

      if (Constants.kInfoMode) {
        Logger.recordOutput("Drivebase/SwerveStateSetpoints", moduleStates);
      }

      // robotRelativeChassisSpeeds =
      //     GeometryUtils.discretize(new ChassisSpeeds(throttle, strafe, rotation));

    } else {
      setSwerveModuleStates(DriveConstants.kXWheels, isOpenLoop);
      if (Constants.kInfoMode) {
        Logger.recordOutput("Drivebase/SwerveStateSetpoints", DriveConstants.kXWheels);
      }
    }
  }

  public void autoDrive(ChassisSpeeds speeds) {
    speeds = GeometryUtils.discretize(speeds); // I dont know if you want this here
    moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setSwerveModuleStates(moduleStates, false);
    // robotRelativeChassisSpeeds = speeds;
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxMetersPerSecond);

    for (int i = 0; i < m_SwerveMods.length; i++) {
      SwerveModule module = m_SwerveMods[i];
      states[i] = new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle);
      module.setDesiredState(states[i], isOpenLoop);
    }
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public SwerveModule getSwerveModule(int moduleNumber) {
    return m_SwerveMods[moduleNumber];
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_SwerveMods[0].getState(),
      m_SwerveMods[1].getState(),
      m_SwerveMods[2].getState(),
      m_SwerveMods[3].getState()
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_SwerveMods[0].getPosition(),
      m_SwerveMods[1].getPosition(),
      m_SwerveMods[2].getPosition(),
      m_SwerveMods[3].getPosition()
    };
  }

  public void updateEstimatorWithPose(Pose2d updatedPose) {
    poseEstimator.resetPosition(getYaw(), getModulePositions(), updatedPose);
  }

  public static SwerveDriveKinematics getSwerveKinematics() {
    return DriveConstants.kDriveKinematics;
  }

  /**
   * Correction for swerve second order dynamics issue. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
  // private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
  //   final double LOOP_TIME_S = 0.02;
  //   Pose2d futureRobotPose =
  //       new Pose2d(
  //           originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
  //           originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
  //           Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
  //   Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
  //   ChassisSpeeds updatedSpeeds =
  //       new ChassisSpeeds(
  //           twistForPose.dx / LOOP_TIME_S,
  //           twistForPose.dy / LOOP_TIME_S,
  //           twistForPose.dtheta / LOOP_TIME_S);
  //   return updatedSpeeds;
  // }

  public Rotation2d getYaw() {
    if (Constants.currentMode == Mode.REAL) {
      return (invertGyro)
          ? Rotation2d.fromDegrees(360 - m_pigeon.getYaw().getValue())
          : Rotation2d.fromDegrees(m_pigeon.getYaw().getValue());
    } else {
      return Rotation2d.fromDegrees(m_simyaw);
    }

  }

  public double getYawDegrees() {
    if (Constants.currentMode == Mode.REAL) {
      return Math.IEEEremainder(m_pigeon.getYaw().getValue(), 360);
    } else {
      return Math.IEEEremainder(m_simyaw, 360);
    }
    
  }

  public void setAngleToSnap(AngleToSnap rotation) {
    m_angleToSnap = rotation;
  }

  public void setSwerveModuleStates(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  // Only used for pathplanner, for some reason they need a chassis speeds supplier
  public ChassisSpeeds getChassisSpeeds() {
    return robotRelativeChassisSpeeds;
  }

  /** Resets each SwerveModule to the absolute position. */
  public void resetAngleToAbsolute() {
    for (SwerveModule mod : m_SwerveMods) {
      mod.resetAngleToAbsolute();
    }
  }

  public void toggleXWheels() {
    if (m_xWheels) {
      m_xWheels = false;
    } else {
      m_xWheels = true;
    }
  }

  public double calculateAngleToSpeaker() {
    double hypot = getPose().getTranslation().getDistance(DriveConstants.kBlueSpeakerPosition);
    double adjacent = getPose().getTranslation().getX() - DriveConstants.kBlueSpeakerPosition.getX();

    return getPose().getY() > DriveConstants.kBlueSpeakerPosition.getY() ? Math.toDegrees(Math.acos(adjacent / hypot)) : -Math.toDegrees(Math.acos(adjacent / hypot));
  }

  @Override
  public void periodic() {
    poseEstimator.update(getYaw(), getModulePositions());

    robotRelativeChassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());

    if (Constants.kInfoMode) {
      Logger.recordOutput(
          "Drivebase/Gyro Connected", !m_pigeon.getFault_BootupGyroscope().getValue());
      Logger.recordOutput("Drivebase/Gyro", getYaw());
      Logger.recordOutput("Drivebase/SwerveStates", getModuleStates());
      Logger.recordOutput("Drivebase/SwervePositions", getModulePositions());
      Logger.recordOutput("Drivebase/Pose", getPose());
      Logger.recordOutput("PathPlanner/Trajectory", GeometryUtils.listToArray(ppPath));
      Logger.recordOutput("PathPlanner/Pathplanner Setpoint", targetPPPose);
      Logger.recordOutput(
          "PathPlanner/PathPlannerError", GeometryUtils.getPoseError(getPose(), targetPPPose));
    }

    m_field.setRobotPose(poseEstimator.getEstimatedPosition());
    m_field.getObject("path").setPoses(ppPath);

    SmartDashboard.putNumber("Angle To Speaker", calculateAngleToSpeaker());

    // if (poseEstimator
    //         .getEstimatedPosition()
    //         .getTranslation()
    //         .getDistance(Limelight.getInstance().getLimelightPose().getTranslation())
    //     <= 2.5 &&
    //     Limelight.getInstance().getLimelightPose().getTranslation().getDistance(new Translation2d(0, 0)) > 1.5) {
    //   poseEstimator.addVisionMeasurement(
    //       Limelight.getInstance().getLimelightPose(),
    //       Timer.getFPGATimestamp() - (Limelight.getInstance().getBotPose()[6] / 1000.0),
    //       VecBuilder.fill(
    //           1
    //               - Math.pow(
    //                   Limelight.getInstance().getA(),
    //                   apriltagTrustMultiplier
    //                       .get()), // Higher the multiplier the closer it has to be to the tag to
    //           // trust it
    //           1 - Math.pow(Limelight.getInstance().getA(), apriltagTrustMultiplier.get()),
    //           0.9));
    // }

  }

  public void realPeriodic() {
    

  }

  @Override
  public void simulationPeriodic() {
    m_simyaw = 
        m_simyaw 
            + Units.radiansToDegrees(
                DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates())
                        .omegaRadiansPerSecond
                    * 0.02);
  }

  public enum AngleToSnap {
    FORWARD(0),
    BACKWARD(180),
    LEFT(90),
    RIGHT(270),
    NONE(0);

    private double angle;

    private AngleToSnap(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return angle;
    }
  }
}
