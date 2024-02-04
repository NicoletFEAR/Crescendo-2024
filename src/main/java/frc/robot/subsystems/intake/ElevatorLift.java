package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.templates.PositionSubsystem;
import frc.robot.subsystems.templates.SubsystemConstants.ManualControlMode;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;

public class ElevatorLift extends PositionSubsystem {

    private static ElevatorLift m_instance = null;

    public ElevatorLift(PositionSubsystemConstants constants) {
        super(constants);
    }

    public static ElevatorLift getInstance() {
        if (m_instance == null) {
            m_instance = new ElevatorLift(ElevatorConstants.kElevatorLiftSubsystemConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
    }

    @Override
    public void outputTelemetry() {}

    public enum ElevatorLiftState implements PositionSubsystemState {
        DOWN(0, 0, "Down"),
        UP(100, 0, "Up"),
        TRANSITION(0, 0, "Transition"),
        MANUAL(0, 0, "Manual");
    
        private double position;
        private double velocity;
        private String name;
    
        private ElevatorLiftState(double position, double velocity, String name) {
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

    public class ElevatorConstants {
        public static final SparkConstants kElevatorLiftLeaderConstants = new SparkConstants();

        static {
          kElevatorLiftLeaderConstants.kID = MotorConstants.kElevatorLiftRightID;
          kElevatorLiftLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
          kElevatorLiftLeaderConstants.kName = "Elevator Lift Right";
          kElevatorLiftLeaderConstants.kIdleMode = IdleMode.kBrake;
          kElevatorLiftLeaderConstants.kMotorType = MotorType.kBrushless;
          kElevatorLiftLeaderConstants.kCurrentLimit = 80;
          kElevatorLiftLeaderConstants.kInverted = false;
          kElevatorLiftLeaderConstants.kKp = 0.1;
          kElevatorLiftLeaderConstants.kKi = 0.0;
          kElevatorLiftLeaderConstants.kKd = 0.0;
        }

        public static final SparkConstants[] kElevatorLiftFollowerConstants = new SparkConstants[1];

        static {
          kElevatorLiftFollowerConstants[0] = new SparkConstants();
          kElevatorLiftFollowerConstants[0].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
          kElevatorLiftFollowerConstants[0].kID = MotorConstants.kElevatorLiftLeftID;
          kElevatorLiftFollowerConstants[0].kName = "Elevator Lift Left";
          kElevatorLiftFollowerConstants[0].kIdleMode = IdleMode.kBrake;
          kElevatorLiftFollowerConstants[0].kMotorType = MotorType.kBrushless;
          kElevatorLiftFollowerConstants[0].kCurrentLimit = 80;
          kElevatorLiftFollowerConstants[0].kInverted = true;
        }

        public static PositionSubsystemConstants kElevatorLiftSubsystemConstants = new PositionSubsystemConstants();

        static {
          // Name of the subsystem, for example "Launcher Flywheels"
          kElevatorLiftSubsystemConstants.kSubsystemName = "Elevator Lift"; 

          // Name of the subsystem, for example "Launcher"
          kElevatorLiftSubsystemConstants.kSuperstructureName = "Intake";

          // An enum which is in the template subsystem
          kElevatorLiftSubsystemConstants.kSubsystemType = PositionSubsystemType.ELEVATOR_LIFT;

          // The main motor constants
          // Instantiate these motor constants above this static block
          kElevatorLiftSubsystemConstants.kLeaderConstants = kElevatorLiftLeaderConstants;

          // An array of motor constants that follow the leader
          // Instantiate these motor constants above this static block
          kElevatorLiftSubsystemConstants.kFollowerConstants = kElevatorLiftFollowerConstants;

          // Initial, Manual, and Transition state of the subsytem
          // This enum is in the Subsystem that extends the MultiMotorPositionSubsystem
          // You will have to create these states
          kElevatorLiftSubsystemConstants.kInitialState = ElevatorLiftState.DOWN;
          kElevatorLiftSubsystemConstants.kManualState = ElevatorLiftState.MANUAL;
          kElevatorLiftSubsystemConstants.kTransitionState = ElevatorLiftState.TRANSITION;

          // Home position of the motor
          kElevatorLiftSubsystemConstants.kHomePosition = 0.0;

          // Conversion factor for the motor output units
          // To find degrees: 360/gear ratio ex 360/100 for 100:1
          // For example for ratio 100:1 do 100
          kElevatorLiftSubsystemConstants.kPositionConversionFactor = 1.0; 

          // Tolerance for atSetpoint()
          kElevatorLiftSubsystemConstants.kSetpointTolerance = 1.0; 

          // PID Slot, make more if more than one set of pid constants are used
          kElevatorLiftSubsystemConstants.kDefaultSlot = 0; 

          // Max velocity and acceleration for trapezoidal motion profile
          kElevatorLiftSubsystemConstants.kMaxVelocity = 100.0; 
          kElevatorLiftSubsystemConstants.kMaxAcceleration = 75.0;

          // Max/Min positions the subsystem should be able to move
          kElevatorLiftSubsystemConstants.kMaxPosition = 100;
          kElevatorLiftSubsystemConstants.kMinPosition = 0;

          // Enum which is found in SubsystemConstants
          kElevatorLiftSubsystemConstants.kManualControlMode = ManualControlMode.LEFT_X;

          // Multiplied by controller inputs
          kElevatorLiftSubsystemConstants.kManualMultiplier = 1;

          // Deadband for controller
          kElevatorLiftSubsystemConstants.kManualDeadBand = .1;
        }

    }
    
}
