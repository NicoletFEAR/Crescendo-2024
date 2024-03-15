// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.subsystems.Limelight;
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

  private double m_simyaw = 0;

  private static final boolean invertGyro = false;

  private SwerveModuleState[] moduleStates;

  private SwerveModule[] m_swerveMods;

  private ChassisSpeeds robotRelativeChassisSpeeds;

  private SwerveDrivePoseEstimator poseEstimator;

  private PoseEstimate limelightPose;

  private boolean isSetGyroRequestPresent;
  
  private boolean isGyroRequestAmpSide;

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


    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            getYaw(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.0),
            VecBuilder.fill(0.9, 0.9, 100.0));

    m_pigeon.setYaw(0);

    robotRelativeChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    // m_snapToAngleController = new PIDController(.06, 0, 0);

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
            5, // Max module speed, in m/s
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

  public void addGyro(double value) {
    if (Constants.kCurrentMode == Mode.REAL) {
      m_pigeon.setYaw(m_pigeon.getYaw().getValue() + value);
    } else {
      m_simyaw += value;
    }
    
  }

  public void drive(double throttle, double strafe, double rotation, boolean isOpenLoop, boolean fieldRelative) {

    if (throttle + strafe + rotation != 0 && m_xWheels == true) {
      m_xWheels = false; // this ends the x wheels when Ari starts moving
    }

    if (m_xWheels == false) {
      throttle = throttle * DriveConstants.kMaxMetersPerSecond;
      strafe = strafe * DriveConstants.kMaxMetersPerSecond;
      rotation = rotation * DriveConstants.kMaxRotationRadiansPerSecond;

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
    return poseEstimator.getEstimatedPosition();
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
    if (Constants.kCurrentMode == Mode.REAL) {
      return (invertGyro)
          ? Rotation2d.fromDegrees(360 - m_pigeon.getYaw().getValue())
          : Rotation2d.fromDegrees(m_pigeon.getYaw().getValue());
    } else {
      return Rotation2d.fromDegrees(m_simyaw);
    }

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
    poseEstimator = estimator;
  }

  public void addVisionEstimate(Pose2d estimate, double timeStamp) {
    poseEstimator.addVisionMeasurement(estimate, timeStamp);
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

  public double calculateDistanceToSpeaker(Pose2d robotPose) {
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return robotPose.getTranslation().getDistance(DriveConstants.kRedSpeakerPosition);
    }
    return robotPose.getTranslation().getDistance(DriveConstants.kBlueSpeakerPosition);
  }

  @Override
  public void periodic() {
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());

    robotRelativeChassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    // SmartDashboard.putNumber("swerve gyro", getYawDegrees());

    if (Constants.kInfoMode) {
      m_field.setRobotPose(poseEstimator.getEstimatedPosition());
    }


    
    // m_field.getObject("path").setPoses(ppPath);

    // SmartDashboard.putNumber("Angle To Speaker", calculateAngleToSpeaker());
    // SmartDashboard.putNumber("Distance To Speaker", calculateDistanceToSpeaker(getPose()));

    // PolarCoordinate[] coord = GeometryUtils.findClosestCoordinates(Math.abs(calculateAngleToSpeaker()), calculateDistanceToSpeaker(getPose()));

    // SmartDashboard.putNumberArray("Top Left Coord", new Double[] {coord[0].getTheta(), coord[0].getR()});

    // SmartDashboard.putNumberArray("Top Right Coord", new Double[] {coord[1].getTheta(), coord[1].getR()});

    // SmartDashboard.putNumberArray("Bottom Left Coord", new Double[] {coord[2].getTheta(), coord[2].getR()});

    // SmartDashboard.putNumberArray("Bottom Right Coord", new Double[] {coord[3].getTheta(), coord[3].getR()});

    // SmartDashboard.putNumber("Interpolated Pitch", GeometryUtils.interpolatePitch(Math.abs(calculateAngleToSpeaker()), calculateDistanceToSpeaker(getPose())));

    // SmartDashboard.putNumber("Pitch", GeometryUtils.interpolatePitch(Math.abs(calculateAngleToSpeaker()), calculateDistanceToSpeaker(getPose())));

    // limelightPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-launch");

    // if (poseEstimator
    //         .getEstimatedPosition()
    //         .getTranslation()
    //         .getDistance(limelightPose.getTranslation())
    //           <= 2.5 && limelightPose.getTranslation().getDistance(new Translation2d(0, 0)) > 0.5) {
    //   poseEstimator.addVisionMeasurement(
    //       limelightPose,
    //       Timer.getFPGATimestamp() - (LimelightHelpers.getBotPose("limelight-launch")[6] / 1000.0));
    // }

    if (Constants.kInfoMode) {
      // SmartDashboard.putBoolean("Is Pose Trustworthy", limelightPose.isPoseTrustworthy());
      // SmartDashboard.putNumber("Bot Ta", LimelightHelpers.getTA("limelight-launch"));
      // m_field.getObject("Limelight-Launch-Pose").setPose(limelightPose.pose);
    }

    // if (limelightPose.isPoseTrustworthy()) {
    //   poseEstimator.addVisionMeasurement(limelightPose.pose, limelightPose.timestampSeconds);
    // }
   
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
