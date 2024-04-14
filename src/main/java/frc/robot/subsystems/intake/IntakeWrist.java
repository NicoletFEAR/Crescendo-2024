package frc.robot.subsystems.intake;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.templates.PositionSubsystem;
import frc.lib.templates.SubsystemConstants.ManualControlMode;
import frc.lib.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.lib.templates.SubsystemConstants.RevMotorType;
import frc.lib.templates.SubsystemConstants.SparkConstants;
import frc.robot.Constants.MotorConstants;

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
    public void outputTelemetry() {}

    public enum IntakeWristState implements PositionSubsystemState {
        DOWN(18, 0),
        STOWED(0, 0),
        AMP(1.20, 0),
        NOTE_TO_LAUNCHER(6, 0),
        TRAVEL(5, 0),
        TRANSITION(0, 0),
        LAUNCHING(6.5, 0),
        MANUAL(0, 0);
    
        private double position;
        private double velocity;
    
        private IntakeWristState(double position, double velocity) {
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

    public class IntakeWristConstants extends PositionSubsystemConstants {
        public static final SparkConstants kIntakeWristLeaderConstants = new SparkConstants();

        static {
            kIntakeWristLeaderConstants.kID = MotorConstants.kIntakeWristID;
            kIntakeWristLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kIntakeWristLeaderConstants.kName = "Intake Wrist Leader";
            kIntakeWristLeaderConstants.kIdleMode = IdleMode.kBrake;
            kIntakeWristLeaderConstants.kMotorType = MotorType.kBrushless;
            kIntakeWristLeaderConstants.kCurrentLimit = 25;
            kIntakeWristLeaderConstants.kInverted = true;
            kIntakeWristLeaderConstants.kKp = 0.23;
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
            kIntakeWristConstants.kSetpointTolerance = .5; 

            // PID Slot, make more if more than one set of pid constants are used
            kIntakeWristConstants.kDefaultSlot = 0; 

            // Max velocity and acceleration for trapezoidal motion profile
            kIntakeWristConstants.kMaxVelocity = 170; 
            kIntakeWristConstants.kMaxAcceleration = 150;

            // Max/Min positions the subsystem should be able to move
            kIntakeWristConstants.kMaxPosition = 18.0;
            kIntakeWristConstants.kMinPosition = 0.0;

            // Enum which is found in SubsystemConstants
            kIntakeWristConstants.kManualControlMode = ManualControlMode.BUMPERS;

            // Multiplied by controller inputs
            kIntakeWristConstants.kManualMultiplier = .2;

            // Deadband for controller
            kIntakeWristConstants.kManualDeadBand = .1;
        }

    }
        
    }
