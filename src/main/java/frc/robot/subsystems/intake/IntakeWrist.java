package frc.robot.subsystems.intake;


import frc.robot.subsystems.templates.SubsystemConstants.ManualControlMode;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.templates.PositionSubsystem;

public class IntakeWrist extends PositionSubsystem {

    private static IntakeWrist m_instance = null;
    // private CANcoder m_intakeWristAbsoluteEncoder;

    public IntakeWrist(PositionSubsystemConstants constants) {
        super(constants);
        // m_intakeWristAbsoluteEncoder = new CANcoder(11);
        // m_intakeWristAbsoluteEncoder.optimizeBusUtilization();
        // m_intakeWristAbsoluteEncoder.getAbsolutePosition().setUpdateFrequency(50);
        
        // // zero(m_intakeWristAbsoluteEncoder.getAbsolutePosition().getValue());
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
        // Logger.recordOutput(m_constants.kSuperstructureName + "/" + m_constants.kSubsystemName + "/Encoder Position", m_intakeWristAbsoluteEncoder.getAbsolutePosition().getValue());
    }

    public enum IntakeWristState implements PositionSubsystemState {
        DOWN(17, 0, "Down"),
        STOWED(0, 0, "Up"),
        AMP(13, 0, "Amp"),
        TRAVEL(3.74, 0, "Travel"),
        TRANSITION(0, 0, "Transition"),
        LAUNCHING(6.5, 0, "Launching"), // this is the angle where the note is minimanlly bent when being passed to the launcher superstructure
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
            kIntakeWristLeaderConstants.kID = MotorConstants.kIntakeWristID;
            kIntakeWristLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kIntakeWristLeaderConstants.kName = "Intake Wrist Leader";
            kIntakeWristLeaderConstants.kIdleMode = IdleMode.kBrake;
            kIntakeWristLeaderConstants.kMotorType = MotorType.kBrushless;
            kIntakeWristLeaderConstants.kCurrentLimit = 20;
            kIntakeWristLeaderConstants.kInverted = true;
            kIntakeWristLeaderConstants.kKp = 0.1;
            kIntakeWristLeaderConstants.kKi = 0.0;
            kIntakeWristLeaderConstants.kKd = 0.0;
        }

        public static final SparkConstants[] kWristFollowerConstants = new SparkConstants[0];

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
            kIntakeWristConstants.kInitialState = IntakeWristState.STOWED;
            kIntakeWristConstants.kManualState = IntakeWristState.MANUAL;
            kIntakeWristConstants.kTransitionState = IntakeWristState.TRANSITION;

            // Home position of the motor
            kIntakeWristConstants.kHomePosition = 0.0;

            // Conversion factor for the motor output units
            // To find degrees: 360/gear ratio ex 360/100 for 100:1
            // For example for ratio 100:1 do 100
            kIntakeWristConstants.kPositionConversionFactor = 1.0; 

            // Tolerance for atSetpoint()
            kIntakeWristConstants.kSetpointTolerance = 1.5; 

            // PID Slot, make more if more than one set of pid constants are used
            kIntakeWristConstants.kDefaultSlot = 0; 

            // Max velocity and acceleration for trapezoidal motion profile
            kIntakeWristConstants.kMaxVelocity = 20; 
            kIntakeWristConstants.kMaxAcceleration = 10;

            // Max/Min positions the subsystem should be able to move
            kIntakeWristConstants.kMaxPosition = 17.0;
            kIntakeWristConstants.kMinPosition = 0.0;

            // Enum which is found in SubsystemConstants
            kIntakeWristConstants.kManualControlMode = ManualControlMode.BUMPERS;

            // Multiplied by controller inputs
            kIntakeWristConstants.kManualMultiplier = .05;

            // Deadband for controller
            kIntakeWristConstants.kManualDeadBand = .1;
        }

    }
        
    }
