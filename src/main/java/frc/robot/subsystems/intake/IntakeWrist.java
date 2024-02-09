package frc.robot.subsystems.intake;


import frc.robot.subsystems.templates.SubsystemConstants.ManualControlMode;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.subsystems.templates.PositionSubsystem;

public class IntakeWrist extends PositionSubsystem {

    private static IntakeWrist m_instance = null;
    private CANcoder m_intakeWristAbsoluteEncoder;

    public IntakeWrist(PositionSubsystemConstants constants) {
        super(constants);
        m_intakeWristAbsoluteEncoder = new CANcoder(11);
        
        // zero(m_intakeWristAbsoluteEncoder.getAbsolutePosition().getValue());
    }

    public static IntakeWrist getInstance() {
        if (m_instance == null) {
            m_instance = new IntakeWrist(IntakeWristConstants.kIntakeWristConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {}

    @Override
    public void outputTelemetry() {
        Logger.recordOutput(m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Encoder Position", m_intakeWristAbsoluteEncoder.getAbsolutePosition().getValue());
    }

    public enum IntakeWristState implements PositionSubsystemState {
        DOWN(5, 0, "Down"),
        UP(0, 0, "Up"),
        AMP(3.5, 0, "Amp"),
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
            kIntakeWristLeaderConstants.kID = 8;
            kIntakeWristLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kIntakeWristLeaderConstants.kName = "Wrist Leader";
            kIntakeWristLeaderConstants.kIdleMode = IdleMode.kBrake;
            kIntakeWristLeaderConstants.kMotorType = MotorType.kBrushless;
            kIntakeWristLeaderConstants.kCurrentLimit = 80;
            kIntakeWristLeaderConstants.kInverted = false;
            kIntakeWristLeaderConstants.kKp = 0.1;
            kIntakeWristLeaderConstants.kKi = 0.0;
            kIntakeWristLeaderConstants.kKd = 0.0;
        }

        public static final SparkConstants[] kWristFollowerConstants = new SparkConstants[1];

        static {
            kWristFollowerConstants[0] = new SparkConstants();
            kWristFollowerConstants[0].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kWristFollowerConstants[0].kID = 7;
            kWristFollowerConstants[0].kName = "Example Follower 1";
            kWristFollowerConstants[0].kIdleMode = IdleMode.kBrake;
            kWristFollowerConstants[0].kMotorType = MotorType.kBrushless;
            kWristFollowerConstants[0].kCurrentLimit = 80;
            kWristFollowerConstants[0].kInverted = true;
        }

        public static PositionSubsystemConstants kIntakeWristConstants = new PositionSubsystemConstants();

        static {
            // Name of the subsystem, for example "Launcher Flywheels"
            kIntakeWristConstants.kSubsystemName = "Intake Wrist"; 

            // Name of the subsystem, for example "Launcher"
            kIntakeWristConstants.kSuperstructureName = "Intake";

            // An enum which is in the template subsystem
            kIntakeWristConstants.kSubsystemType = PositionSubsystemType.INTAKE_WRIST;

            // The main motor constants
            // Instantiate these motor constants above this static block
            kIntakeWristConstants.kLeaderConstants = kIntakeWristLeaderConstants;

            // An array of motor constants that follow the leader
            // Instantiate these motor constants above this static block
            kIntakeWristConstants.kFollowerConstants = kWristFollowerConstants;

            // Initial, Manual, and Transition state of the subsytem
            // This enum is in the Subsystem that extends the MultiMotorPositionSubsystem
            // You will have to create these states
            kIntakeWristConstants.kInitialState = IntakeWristState.DOWN;
            kIntakeWristConstants.kManualState = IntakeWristState.MANUAL;
            kIntakeWristConstants.kTransitionState = IntakeWristState.TRANSITION;

            // Home position of the motor
            kIntakeWristConstants.kHomePosition = 0.0;

            // Conversion factor for the motor output units
            // To find degrees: 360/gear ratio ex 360/100 for 100:1
            // For example for ratio 100:1 do 100
            kIntakeWristConstants.kPositionConversionFactor = 1.0; 

            // Tolerance for atSetpoint()
            kIntakeWristConstants.kSetpointTolerance = 0.1; 

            // PID Slot, make more if more than one set of pid constants are used
            kIntakeWristConstants.kDefaultSlot = 0; 

            // Max velocity and acceleration for trapezoidal motion profile
            kIntakeWristConstants.kMaxVelocity = 10; 
            kIntakeWristConstants.kMaxAcceleration = 20;

            // Max/Min positions the subsystem should be able to move
            kIntakeWristConstants.kMaxPosition = Double.POSITIVE_INFINITY;
            kIntakeWristConstants.kMinPosition = Double.NEGATIVE_INFINITY;

            // Enum which is found in SubsystemConstants
            kIntakeWristConstants.kManualControlMode = ManualControlMode.TRIGGERS;

            // Multiplied by controller inputs
            kIntakeWristConstants.kManualMultiplier = 1;

            // Deadband for controller
            kIntakeWristConstants.kManualDeadBand = .1;
        }

    }
        
    }