// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
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
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.lib.utilities.GeometryUtils;
import frc.lib.utilities.LimelightHelpers;
import frc.lib.utilities.SwerveModuleConstants;
import frc.lib.utilities.LimelightHelpers.PoseEstimate;

import java.util.ArrayList;
import java.util.List;

public class SwerveDrive extends SubsystemBase {

  private static SwerveDrive m_instance = null;

  private final Pigeon2 m_pigeon = new Pigeon2(Constants.DriveConstants.kPigeon, "CANivore");

  // private Pose2d targetPPPose = new Pose2d(0, 0, new Rotation2d(0));
  public static List<Pose2d> ppPath = new ArrayList<>();

  public final Field2d m_field = new Field2d();

  private boolean m_xWheels = false;
  private boolean m_targetNote = false;
  private boolean m_trackSpeaker = false;

  private double kff = 0;

  private PIDController m_trackNoteController;
  private PIDController m_trackSpeakerController;

  private double m_simyaw = 0;

  private static final boolean invertGyro = false;

  private SwerveModuleState[] moduleStates;

  private SwerveModule[] m_swerveMods;

  private ChassisSpeeds robotRelativeChassisSpeeds;

  private SwerveDrivePoseEstimator m_poseEstimator;

  // private PoseEstimate limelightPose;

  private boolean isSetGyroRequestPresent;
  
  private boolean isGyroRequestAmpSide;

  private PoseEstimate m_poseEstimate;

  private boolean intakeReady;

  private boolean enableAutoVision;

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

    m_swerveMods =
        new SwerveModule[] {
          new SwerveModule(0, frontLeftModuleConstants),
          new SwerveModule(1, frontRightModuleConstants),
          new SwerveModule(2, backLeftModuleConstants),
          new SwerveModule(3, backRightModuleConstants)
        };

    m_pigeon.optimizeBusUtilization();
    m_pigeon.getYaw().setUpdateFrequency(100);

    /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more.
     */
    Timer.delay(1.0);
    resetAngleToAbsolute();

    for (SwerveModule module : m_swerveMods) {
      module.burnFlash();
    }


    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.0),
            VecBuilder.fill(0.9, 0.9, 9999999));

    m_pigeon.setYaw(0);

    robotRelativeChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    m_trackNoteController = new PIDController(.01, 0, 0.0);
    m_trackSpeakerController = new PIDController(.016, 0.001, 0.0);
    // RobotContainer.mainTab.add("Turn To Note Controller", m_trackNoteController);
    // RobotContainer.mainTab.add("Turn To Speaker While Driving Controller", m_trackSpeakerController);

    if (Constants.kInfoMode) {
      RobotContainer.mainTab.add(m_field).withPosition(2, 0).withSize(8, 5);
    }

    // PathPlannerLogging.setLogTargetPoseCallback(
    //     (targetPose) -> {
    //       targetPPPose = targetPose;
    //     });

    // PathPlannerLogging.setLogActivePathCallback(
    //     (activePath) -> {
    //       ppPath = activePath;
    //     });

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
            5.5, // Max module speed, in m/s
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

  public void setGyro(double value) {
    m_pigeon.setYaw(value);
  }

  public void setAutoVision(boolean value) {
    enableAutoVision = value;
  }

  public void addGyro(double value) {
    if (Constants.kCurrentMode == Mode.REAL) {
      m_pigeon.setYaw(m_pigeon.getYaw().getValue() + value);
    } else {
      m_simyaw += value;
    }
    
  }

  public void drive(double throttle, double strafe, double rotation, boolean isOpenLoop, boolean fieldRelative) {

    SmartDashboard.putNumber("Inputed Rotation", rotation);

    if (throttle + strafe + rotation != 0 && m_xWheels == true) {
      m_xWheels = false; // this ends the x wheels when Ari starts moving
    }

    if (!m_xWheels) {
      if (m_targetNote) {
        strafe = 0;
        throttle = Math.abs(throttle * DriveConstants.kMaxMetersPerSecond);
        rotation = MathUtil.clamp(m_trackNoteController.calculate(LimelightHelpers.getTX("limelight-intake"), 0), -1, 1);
        rotation *= DriveConstants.kMaxRotationRadiansPerSecond;
      } else if (m_trackSpeaker) {
        throttle = throttle * DriveConstants.kMaxMetersPerSecond;
        strafe = strafe * DriveConstants.kMaxMetersPerSecond;

        double m_targetAngle;

        m_targetAngle = calculateAngleToSpeaker() < 0 ? calculateAngleToSpeaker() + 180 : calculateAngleToSpeaker() - 180;

        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
          m_targetAngle += -5;
        } else {
          m_targetAngle -= 185;
        }

        kff = SmartDashboard.getNumber("Turn To Angle KFF", kff);

        double speeds =
            m_trackSpeakerController.calculate(
                GeometryUtils.getAdjustedYawDegrees(getYawDegrees(), m_targetAngle), 180);
    
        kff = speeds > 0.0 ? Math.abs(kff) : -Math.abs(kff);
    
        speeds = MathUtil.clamp(speeds + kff, -1, 1);

        rotation = speeds * DriveConstants.kMaxRotationRadiansPerSecond;
      } else {
        throttle = throttle * DriveConstants.kMaxMetersPerSecond;
        strafe = strafe * DriveConstants.kMaxMetersPerSecond;
        rotation = rotation * DriveConstants.kMaxRotationRadiansPerSecond;
      }
      

      ChassisSpeeds chassisSpeeds =
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, getYaw())
              : new ChassisSpeeds(throttle, strafe, rotation);

      // chassisSpeeds = GeometryUtils.discretize(chassisSpeeds);
      moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      setSwerveModuleStates(moduleStates, isOpenLoop);

      // robotRelativeChassisSpeeds =
      //     GeometryUtils.discretize(new ChassisSpeeds(throttle, strafe, rotation));

    } else {
      setSwerveModuleStates(DriveConstants.kXWheels, isOpenLoop);
    }
  }

  public void setIntakeReady(boolean value) {
    intakeReady = value;
  }

  public boolean getIntakeReady() {
    return intakeReady;
  }

  public void autoDrive(ChassisSpeeds speeds) {
    speeds = GeometryUtils.discretize(speeds); // I dont know if you want this here
    moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setSwerveModuleStates(moduleStates, false);
    // robotRelativeChassisSpeeds = speeds;
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxMetersPerSecond);

    for (int i = 0; i < m_swerveMods.length; i++) {
      SwerveModule module = m_swerveMods[i];
      states[i] = new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle);
      module.setDesiredState(states[i], isOpenLoop);
    }
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public SwerveModule getSwerveModule(int moduleNumber) {
    return m_swerveMods[moduleNumber];
  }

  public SwerveModule[] getSwerveModules() {
    return m_swerveMods;
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_swerveMods[0].getState(),
      m_swerveMods[1].getState(),
      m_swerveMods[2].getState(),
      m_swerveMods[3].getState()
    };
  }

  public SwerveModulePosition[] getModulePositions() {
    // if (m_swerveMods[0].getDriveMetersPerSecond() + m_swerveMods[1].getDriveMetersPerSecond() +
    // m_swerveMods[2].getDriveMetersPerSecond() + m_swerveMods[3].getDriveMetersPerSecond() < 32) {
      return new SwerveModulePosition[] {
        m_swerveMods[0].getPosition(),
        m_swerveMods[1].getPosition(),
        m_swerveMods[2].getPosition(),
        m_swerveMods[3].getPosition()
      };
    // }

  }

  public void updateEstimatorWithPose(Pose2d updatedPose) {
    m_poseEstimator.resetPosition(getYaw(), getModulePositions(), updatedPose);
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
    if (Constants.kCurrentMode == Mode.REAL) {
      return (invertGyro)
          ? Rotation2d.fromDegrees(360 - m_pigeon.getYaw().getValue())
          : Rotation2d.fromDegrees(m_pigeon.getYaw().getValue());
    } else {
      return Rotation2d.fromDegrees(m_simyaw);
    }

  }

  public Pigeon2 getPigeon() {
    return m_pigeon;
  }

  public double getYawDegrees() {
    if (Constants.kCurrentMode == Mode.REAL) {
      return Math.IEEEremainder(m_pigeon.getYaw().getValue(), 360);
    } else {
      return Math.IEEEremainder(m_simyaw, 360);
    }
    
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
    for (SwerveModule mod : m_swerveMods) {
      mod.resetAngleToAbsolute();
    }
  }

  public void toggleXWheels() {
    if (m_xWheels) {
      m_xWheels = false;
    } else {
      m_xWheels = true;
    }
    // System.out.println("toggled");
  }

  public void setNoteTracking(boolean value) {
    m_targetNote = value;
  }

  public void setSpeakerTracking(boolean value) {
    m_trackSpeaker = value;
  }

  public boolean getXWheels() {
    return m_xWheels;
  }

  public boolean getIsSetGyroRequestPresent() {
    return isSetGyroRequestPresent;
  }

  public boolean getIsGyroRequestAmpSide() {
    return isGyroRequestAmpSide;
  }

  public void setGyroRequest(boolean requestPresent, boolean isAmpSide) {
    isSetGyroRequestPresent = requestPresent;
    isGyroRequestAmpSide = isAmpSide;
  }

  public void resetPoseEstimator(SwerveDrivePoseEstimator estimator) {
    m_poseEstimator = estimator;
  }

  public void addVisionEstimate(Pose2d estimate, double timeStamp) {
    m_poseEstimator.addVisionMeasurement(estimate, timeStamp);
  }

  public double calculateAngleToSpeaker() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      double hypot = getPose().getTranslation().getDistance(DriveConstants.kRedSpeakerPosition);
      double adjacent = getPose().getTranslation().getX() - DriveConstants.kRedSpeakerPosition.getX();

      if(getPose().getY() > DriveConstants.kRedSpeakerPosition.getY()){
        return Math.toDegrees(Math.acos(adjacent / hypot));
      }
      else{
        return -Math.toDegrees(Math.acos(adjacent / hypot));
      }
    } else {
      double hypot = getPose().getTranslation().getDistance(DriveConstants.kBlueSpeakerPosition);
      double adjacent = getPose().getTranslation().getX() - DriveConstants.kBlueSpeakerPosition.getX();

      if(getPose().getY() > DriveConstants.kBlueSpeakerPosition.getY()){
        return Math.toDegrees(Math.acos(adjacent / hypot));
      }
      else{
        return -Math.toDegrees(Math.acos(adjacent / hypot));
      }
    }

  }

  public double calculateAngleToAmp() {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      double hypot = getPose().getTranslation().getDistance(DriveConstants.kRedAmpPassPosition);
      double adjacent = getPose().getTranslation().getX() - DriveConstants.kRedAmpPassPosition.getX();

      if(getPose().getY() > DriveConstants.kRedAmpPassPosition.getY()){
        return Math.toDegrees(Math.acos(adjacent / hypot));
      }
      else{
        return -Math.toDegrees(Math.acos(adjacent / hypot));
      }
    } else {
      double hypot = getPose().getTranslation().getDistance(DriveConstants.kBlueAmpPassPosition);
      double adjacent = getPose().getTranslation().getX() - DriveConstants.kBlueAmpPassPosition.getX();

      if(getPose().getY() > DriveConstants.kBlueAmpPassPosition.getY()){
        return Math.toDegrees(Math.acos(adjacent / hypot));
      }
      else{
        return -Math.toDegrees(Math.acos(adjacent / hypot));
      }
    }

  }

  public double calculateDistanceToSpeaker(Pose2d robotPose) {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return robotPose.getTranslation().getDistance(DriveConstants.kRedSpeakerPosition);
    }
    return robotPose.getTranslation().getDistance(DriveConstants.kBlueSpeakerPosition);
  }

  @Override
  public void periodic() {
    m_poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());

    robotRelativeChassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());

    SmartDashboard.putNumber("Gyro", getYawDegrees());

    LimelightHelpers.SetRobotOrientation("limelight-launch", getYaw().getDegrees(), 0, 0, 0, 0, 0);
    m_poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-launch");

    if (Math.abs(m_pigeon.getRate()) < 720 && !DriverStation.isAutonomous() && m_poseEstimate.pose.getX() != 0) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.6,.6,9999999));
      m_poseEstimator.addVisionMeasurement(m_poseEstimate.pose, m_poseEstimate.timestampSeconds);
    } else if (enableAutoVision && m_poseEstimate.isPoseTrustworthy()) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.6,.6,9999999));
      m_poseEstimator.addVisionMeasurement(m_poseEstimate.pose, m_poseEstimate.timestampSeconds);
    }

    if (Constants.kInfoMode) {
      m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
      m_field.getObject("Limelight-Pose").setPose(m_poseEstimate.pose);
      SmartDashboard.putNumber("Distance From Speaker", calculateDistanceToSpeaker(getPose()));
      SmartDashboard.putNumber("Angle From Speaker", calculateAngleToSpeaker());
    }
   
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
