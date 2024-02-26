package frc.robot.subsystems.launcher;

import frc.robot.subsystems.templates.SubsystemConstants.ManualControlMode;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.utilities.GeometryUtils;
import frc.robot.RobotContainer;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.templates.PositionSubsystem;

public class LauncherWrist extends PositionSubsystem {

    private static LauncherWrist m_instance = null;

    // private CANcoder m_launcherWristAbsoluteEncoder;

    // private ArmFeedforward m_feedforward;

    public LauncherWrist(PositionSubsystemConstants constants) {
        super(constants);

        // m_launcherWristAbsoluteEncoder = new CANcoder(LauncherConstants.kWristCANCoderId);
        // m_launcherWristAbsoluteEncoder.optimizeBusUtilization();
        // m_launcherWristAbsoluteEncoder.getAbsolutePosition().setUpdateFrequency(50);

        // m_feedforward = new ArmFeedforward(m_constants.kLeaderConstants.kKs, m_constants.kLeaderConstants.kKg, m_constants.kLeaderConstants.kKv);


   

        // zero(m_launcherWristAbsoluteEncoder.getAbsolutePosition().getValue());
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

        // SmartDashboard.putNumber("Wrist position", m_encoder.getPosition());
        // SmartDashboard.putNumber("Intended Position", m_desiredState.getPosition());

        // setFeedforward(m_feedforward.calculate(m_setpoint.position, m_setpoint.velocity));
    }

    @Override
    public void outputTelemetry() {}

    public double calculatePitch() {
        // double distance = SwerveDrive.getInstance().getPose().getTranslation().getDistance(DriveConstants.kBlueSpeakerPosition);

        // if (distance > 0 && distance < LauncherConstants.kDistancePitchMap.lastKey()) {
        //     double lowerPitch = LauncherConstants.kDistancePitchMap.get(LauncherConstants.kDistancePitchMap.floorKey(distance));
        //     double upperPitch = LauncherConstants.kDistancePitchMap.get(LauncherConstants.kDistancePitchMap.ceilingKey(distance));
        //     return lowerPitch + (distance - Math.floor(distance)) * (upperPitch - lowerPitch);
        // } else {
        //     return 0;
        // }

        return 0;
    }

    public enum LauncherWristState implements PositionSubsystemState {
        DOWN(0, 0, "Down"),
        UP(45, 0, "Up"),
        SUBWOOFER(100, 0, "Subwoofer"), // use when against base of speaker
        WING_NOTE_1(56.66, 0, "Wing Note 1"),
        WING_NOTE_2(56.66, 0, "Wing Note 2"),
        WING_NOTE_3(56.66, 0, "Wing Note 3"),
        PODIUM(56.66, 0, "podium"), // use when against base of podium
        FIELD_BASED_PITCH(0, 0, "Field Based Pitch"),
        TRANSITION(0, 0, "Transition"),
        LAUNCH(50, 0, "Launch"), // this is the arbitrary value used for testing
        MANUAL(0, 0, "Manual");
    
        private double position;
        private double velocity;
        private String name;
    
        private LauncherWristState(double position, double velocity, String name) {
          this.position = position;
          this.velocity = velocity;
          this.name = name;
        }

        @Override
        public String getName() {
            return name;
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
            kLauncherWristLeaderConstants.kCurrentLimit = 80;
            kLauncherWristLeaderConstants.kInverted = false;
            kLauncherWristLeaderConstants.kKp = 0.07;
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
            kLauncherWristConstants.kPositionConversionFactor = 360/100;

            kLauncherWristConstants.kSetpointTolerance = .25;

            kLauncherWristConstants.kDefaultSlot = 0;

            kLauncherWristConstants.kMaxVelocity = 300;
            kLauncherWristConstants.kMaxAcceleration = 275;

            kLauncherWristConstants.kMaxPosition = 100;
            kLauncherWristConstants.kMinPosition = 20;

            kLauncherWristConstants.kManualControlMode = ManualControlMode.LEFT_X;
            kLauncherWristConstants.kManualMultiplier = 1;
            kLauncherWristConstants.kManualDeadBand = 0.1;

            kLauncherWristConstants.kInitialState = LauncherWristState.UP;
            kLauncherWristConstants.kManualState = LauncherWristState.MANUAL;
            kLauncherWristConstants.kTransitionState = LauncherWristState.TRANSITION;
        }
    }
    
}
