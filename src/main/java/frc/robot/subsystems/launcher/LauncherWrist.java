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
        
        if (m_currentState == LauncherWristState.FIELD_BASED_PITCH) {
            holdPosition();
        }
    }

    @Override
    public void outputTelemetry() {

    }

    public enum LauncherWristState implements PositionSubsystemState {
        DOWN(0, 0, "Down"),
        UP(0, 0, "Up"),
        SUBWOOFER(110, 0, "Subwoofer"), // use when against base of speaker
        WING_NOTE_1(56.66, 0, "Wing Note 1"),
        WING_NOTE_2(56.66, 0, "Wing Note 2"),
        WING_NOTE_3(56.66, 0, "Wing Note 3"),
        LAUNCH_POS_1(56.66, 0, "Launch Pos 1"),
        LAUNCH_POS_2(56.66, 0, "Launch Pos 2"),
        LAUNCH_POS_3(56.66, 0, "Launch Pos 3"),
        POOP_POS_1(56.66, 0, "Poop Pos 1"),
        POOP_POS_2(56.66, 0, "Poop Pos 2"),
        POOP_POS_3(56.66, 0, "Poop Pos 3"),
        PODIUM(45.66, 0, "podium"), // use when against base of podium
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
            kLauncherWristLeaderConstants.kKp = 0.0375;
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

            kLauncherWristConstants.kInitialState = LauncherWristState.UP;
            kLauncherWristConstants.kManualState = LauncherWristState.MANUAL;
            kLauncherWristConstants.kTransitionState = LauncherWristState.TRANSITION;
        }
    }
    
}
