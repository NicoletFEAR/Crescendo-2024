package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.templates.PositionSubsystem;
import frc.lib.templates.SubsystemConstants.ManualControlMode;
import frc.lib.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.lib.templates.SubsystemConstants.RevMotorType;
import frc.lib.templates.SubsystemConstants.SparkConstants;
import frc.lib.utilities.GeometryUtils;
import frc.robot.RobotContainer;
import frc.robot.Constants.MotorConstants;

public class LauncherWrist extends PositionSubsystem {

    private static LauncherWrist m_instance = null;

    public LauncherWrist(PositionSubsystemConstants constants) {
        super(constants);
    }

    public static LauncherWrist getInstance() {
        if (m_instance == null) {
            m_instance = new LauncherWrist(LauncherWristConstants.kLauncherWristConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
        LauncherWristState.FIELD_BASED_PITCH.setPosition(GeometryUtils.interpolatePitch(
            Math.abs(RobotContainer.m_drivebase.calculateAngleToSpeaker()),
            RobotContainer.m_drivebase.calculateDistanceToSpeaker(RobotContainer.m_drivebase.getPose())));
            
        // LauncherWristState.PASS.setPosition(calculateAmpPitch());
        
        SmartDashboard.putNumber("Field Relative Pitch", GeometryUtils.interpolatePitch(
            Math.abs(RobotContainer.m_drivebase.calculateAngleToSpeaker()),
            RobotContainer.m_drivebase.calculateDistanceToSpeaker(RobotContainer.m_drivebase.getPose())));
        
        if ((m_currentState == LauncherWristState.FIELD_BASED_PITCH && m_desiredState == LauncherWristState.FIELD_BASED_PITCH)) {
            holdPosition();
        }

        if (LauncherWristState.MANUAL.getPosition() != 0) {
            LauncherWristState.TESTING.setPosition(LauncherWristState.MANUAL.getPosition());
        }
    }

    // private double calculateAmpPitch() {
    //     double distance;

    //     if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
    //         distance = RobotContainer.m_drivebase.getPose().getTranslation().getDistance(DriveConstants.kRedAmpPassPosition);
    //     } else {
    //         distance = RobotContainer.m_drivebase.getPose().getTranslation().getDistance(DriveConstants.kBlueAmpPassPosition);
    //     }
        
    //     if (distance > 0 && distance < LauncherConstants.kAmpDistancePitchMap.lastKey()) {
    //         double lowerRPM = LauncherConstants.kAmpDistancePitchMap.get(LauncherConstants.kAmpDistancePitchMap.floorKey(distance));
    //         double upperRPM = LauncherConstants.kAmpDistancePitchMap.get(LauncherConstants.kAmpDistancePitchMap.ceilingKey(distance));
    //         return lowerRPM + (distance - Math.floor(distance)) * (upperRPM - lowerRPM);
    //     } else {
    //         return 0;
    //     }
    // }

    @Override
    public void outputTelemetry() {

    }

    public enum LauncherWristState implements PositionSubsystemState {
        DOWN(0, 0),
        UP(65, 0),
        IDLE(65, 0),
        SUBWOOFER(115, 0), // use when against base of speaker
        WING_NOTE_1(85, 0),
        WING_NOTE_2(93, 0),
        WING_NOTE_3(90, 0),
        LAUNCH_POS_1(56.66, 0),
        LAUNCH_POS_2(56.66, 0),
        LAUNCH_POS_3(56.66, 0),
        PASS(58, 0),
        POOP_POS_1(0, 0),
        POOP_POS_2(0, 0),
        POOP_POS_3(0, 0),
        PODIUM(68.3, 0), // use when against base of podium
        FIELD_BASED_PITCH(0, 0),
        TRANSITION(0, 0),
        TESTING(0, 0),
        MANUAL(0, 0);
    
        private double position;
        private double velocity;
    
        private LauncherWristState(double position, double velocity) {
          this.position = position;
          this.velocity = velocity;
        }

        @Override
        public String getName() {
            return name();
        }

        @Override
        public double getVelocity() {
            return velocity;
        }

        @Override
        public void setVelocity(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public void setPosition(double position) {
            this.position = position;
        }
    }

    public class LauncherWristConstants {

        public static final SparkConstants kLauncherWristLeaderConstants = new SparkConstants();

        static {
            kLauncherWristLeaderConstants.kID = MotorConstants.kLauncherWristID;
            kLauncherWristLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kLauncherWristLeaderConstants.kIdleMode = IdleMode.kBrake;
            kLauncherWristLeaderConstants.kMotorType = MotorType.kBrushless;
            kLauncherWristLeaderConstants.kCurrentLimit = 20; //80;
            kLauncherWristLeaderConstants.kInverted = false;
            kLauncherWristLeaderConstants.kKp = 0.06;
            kLauncherWristLeaderConstants.kKi = 0.0;
            kLauncherWristLeaderConstants.kKd = 0.0;
            kLauncherWristLeaderConstants.kKff = 0.0;
        }

        public static final SparkConstants[] kWristFollowerConstants = new SparkConstants[0];

        public static final PositionSubsystemConstants kLauncherWristConstants =
            new PositionSubsystemConstants();

        static {
            kLauncherWristConstants.kSubsystemName = "Launcher Wrist";
            kLauncherWristConstants.kSuperstructureName = "Launcher";

            kLauncherWristConstants.kSubsystemType = PositionSubsystemType.LAUNCHER_WRIST;

            kLauncherWristConstants.kLeaderConstants = kLauncherWristLeaderConstants;
            kLauncherWristConstants.kFollowerConstants = kWristFollowerConstants;

            kLauncherWristConstants.kHomePosition = 0;
            kLauncherWristConstants.kPositionConversionFactor = 360/25;

            kLauncherWristConstants.kSetpointTolerance = .25;

            kLauncherWristConstants.kDefaultSlot = 0;

            kLauncherWristConstants.kMaxVelocity = 1250; // 300
            kLauncherWristConstants.kMaxAcceleration = 1500; //275

            kLauncherWristConstants.kMaxPosition = 110;
            kLauncherWristConstants.kMinPosition = 0;

            kLauncherWristConstants.kManualControlMode = ManualControlMode.LEFT_X;
            kLauncherWristConstants.kManualMultiplier = .75;
            kLauncherWristConstants.kManualDeadBand = 0.1;

            kLauncherWristConstants.kInitialState = LauncherWristState.DOWN;
            kLauncherWristConstants.kManualState = LauncherWristState.MANUAL;
            kLauncherWristConstants.kTransitionState = LauncherWristState.TRANSITION;
        }
    }
    
}
