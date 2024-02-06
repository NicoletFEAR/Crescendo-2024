package frc.robot.subsystems.intake;


import frc.robot.subsystems.templates.SubsystemConstants.ManualControlMode;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;
import frc.robot.subsystems.templates.SubsystemConstants.VoltageSubsystemConstants;
import frc.robot.subsystems.templates.VelocitySubsystem.VelocitySubsystemType;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.templates.PositionSubsystem;

public class IntakeWrist extends PositionSubsystem {

    private static IntakeWrist m_instance = null;

    public IntakeWrist(PositionSubsystemConstants constants) {
        super(constants);
    }

    public static IntakeWrist getInstance() {
        if (m_instance == null) {
            m_instance = new IntakeWrist(IntakeWristConstants.kIntakeWristConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
        IntakeWristState.FIELD_BASED_PITCH.setPosition(calculatePitch());
        SmartDashboard.putNumber("Current Wrist Pitch", m_currentState.getPosition());
    }

    @Override
    public void outputTelemetry() {}

    public double calculatePitch() {
        // double distance = SwerveDrive.getInstance().getPose().getTranslation().getDistance(DriveConstants.kBlueSpeakerPosition);

        // if (distance > 0 && distance < IntakeConstants.kDistancePitchMap.lastKey()) {
        //     double lowerPitch = IntakeConstants.kDistancePitchMap.get(IntakeConstants.kDistancePitchMap.floorKey(distance));
        //     double upperPitch = IntakeConstants.kDistancePitchMap.get(IntakeConstants.kDistancePitchMap.ceilingKey(distance));
        //     return lowerPitch + (distance - Math.floor(distance)) * (upperPitch - lowerPitch);
        // } else {
        //     return 0;
        // }

        return 0;
    }

    public enum IntakeWristState implements PositionSubsystemState {
        DOWN(5, 0, "Down"),
        UP(0, 0, "Up"),
        AMP(3.5, 0, "Amp"),
        FIELD_BASED_PITCH(0, 0, "Field Based Pitch"),
        TRANSITION(0, 0, "Transition"),
        MANUAL(0, 0, "Manual");
    
        private double position;
        private double velocity;
        private String name;
    
        private IntakeWristState(double position, double velocity, String name) {
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

    public class IntakeWristConstants extends PositionSubsystemConstants {
        public static final SparkConstants kIntakeWristLeaderConstants = new SparkConstants();
        static {
            kIntakeWristLeaderConstants.kID = MotorConstants.kIntakeWristLeftID;
            kIntakeWristLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kIntakeWristLeaderConstants.kName = "Intake Wrist Left Leader";
            kIntakeWristLeaderConstants.kIdleMode = IdleMode.kBrake;
            kIntakeWristLeaderConstants.kMotorType = MotorType.kBrushless;
            kIntakeWristLeaderConstants.kCurrentLimit = 80;
            kIntakeWristLeaderConstants.kInverted = false;
            kIntakeWristLeaderConstants.kKp = 0.00001;
            kIntakeWristLeaderConstants.kKi = 0.0;
            kIntakeWristLeaderConstants.kKd = 0.0;
            kIntakeWristLeaderConstants.kKff = 0.0001675;
        }

        public static final SparkConstants[] kIntakeWristFollowerConstants = new SparkConstants[1];
        static {
            kIntakeWristFollowerConstants[0].kID = MotorConstants.kIntakeWristRightID;
            kIntakeWristFollowerConstants[0].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kIntakeWristFollowerConstants[0].kName = "Intake Wrist Right Follower";
            kIntakeWristFollowerConstants[0].kIdleMode = IdleMode.kBrake;
            kIntakeWristFollowerConstants[0].kMotorType = MotorType.kBrushless;
            kIntakeWristFollowerConstants[0].kCurrentLimit = 80;
            kIntakeWristFollowerConstants[0].kInverted = true;
            kIntakeWristFollowerConstants[0].kKp = 0.00001;
            kIntakeWristFollowerConstants[0].kKi = 0.0;
            kIntakeWristFollowerConstants[0].kKd = 0.0;
            kIntakeWristFollowerConstants[0].kKff = 0.0001675;
        }
        
        // Subsystem Constants \\
        public static final PositionSubsystemConstants kIntakeWristConstants = new PositionSubsystemConstants();
        static {
            kIntakeWristConstants.kSubsystemName = "Intake Wrist";
            kIntakeWristConstants.kSuperstructureName = "Intake";

            kIntakeWristConstants.kSubsystemType = PositionSubsystemType.INTAKE_WRIST;

            kIntakeWristConstants.kLeaderConstants = kIntakeWristLeaderConstants;
            kIntakeWristConstants.kFollowerConstants = kIntakeWristFollowerConstants;

            kIntakeWristConstants.kInitialState = null;
            kIntakeWristConstants.kManualState = null;
            kIntakeWristConstants.kTransitionState = null;

            // Servo Motor Subsystem Constants \\
            kIntakeWristConstants.kHomePosition = 0.0;
            kIntakeWristConstants.kPositionConversionFactor =
                1.0; // To find degrees: 360/gear ration ex 360/100 for 100:1

            kIntakeWristConstants.kSetpointTolerance = 0.0; // Tolerance for atSetpoint()

            kIntakeWristConstants.kDefaultSlot =
                0; // PID Slot, make more if more than one set of pid constants are used

            kIntakeWristConstants.kMaxVelocity = 0.0; // Max velocity for motion profile
            kIntakeWristConstants.kMaxAcceleration = 0.0; // Max acceleration for motion profile

            // Max/Min positions the subsystem should be able to move
            kIntakeWristConstants.kMaxPosition = Double.POSITIVE_INFINITY;
            kIntakeWristConstants.kMinPosition = Double.NEGATIVE_INFINITY;

            // Manual constants
            kIntakeWristConstants.kManualControlMode = ManualControlMode.RIGHT_Y;
            kIntakeWristConstants.kManualMultiplier = 0;
            kIntakeWristConstants.kManualDeadBand = 0;
        }
    }
        
    }