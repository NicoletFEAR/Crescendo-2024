// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.templates.PositionSubsystem;
import frc.robot.subsystems.templates.SubsystemConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;
import frc.robot.subsystems.templates.SubsystemConstants.PositionSubsystemConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimbWinch extends PositionSubsystem {
    private static ClimbWinch m_instance = null;

    /** Creates a new ClimbSubsystem. */
    public ClimbWinch(PositionSubsystemConstants constants) {
        super(constants);
    }
  
    public static ClimbWinch getInstance() {
        if (m_instance == null) {
            m_instance = new ClimbWinch(ClimbWinchConstants.kClimbWinchConstants);
        }

        return m_instance;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    @Override
    public void subsystemPeriodic() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'subsystemPeriodic'");
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
    }

    public enum ClimbWinchState implements PositionSubsystemState {
        HIGH(0, 0, "Down"),
        LOW(100, 0, "Up"),
        AMP(75, 0, "Amp"),
        TRANSITION(0, 0, "Transition"),
        MANUAL(0, 0, "Manual");

        private double position;
        private double velocity;
        private String name;

        private ClimbWinchState(double position, double velocity, String name) {
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

    public class ClimbWinchConstants extends PositionSubsystemConstants {
        public static final SparkConstants kClimbWinchLeaderConstants = new SparkConstants();

        static {
            kClimbWinchLeaderConstants.kID = MotorConstants.kClimbWinchLeftID;
            kClimbWinchLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kClimbWinchLeaderConstants.kName = "Climb Winch Leader";
            kClimbWinchLeaderConstants.kIdleMode = IdleMode.kBrake;
            kClimbWinchLeaderConstants.kMotorType = MotorType.kBrushless;
            kClimbWinchLeaderConstants.kCurrentLimit = 80;
            kClimbWinchLeaderConstants.kInverted = false;
            kClimbWinchLeaderConstants.kKp = 0.1;
            kClimbWinchLeaderConstants.kKi = 0.0;
            kClimbWinchLeaderConstants.kKd = 0.0;
        }

        public static final SparkConstants[] kWristFollowerConstants = new SparkConstants[1];

        static {
            kWristFollowerConstants[0] = new SparkConstants();
            kWristFollowerConstants[0].kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kWristFollowerConstants[0].kID = MotorConstants.kClimbWinchRightID;
            kWristFollowerConstants[0].kName = "Climb Winch Follower";
            kWristFollowerConstants[0].kIdleMode = IdleMode.kBrake;
            kWristFollowerConstants[0].kMotorType = MotorType.kBrushless;
            kWristFollowerConstants[0].kCurrentLimit = 80;
            kWristFollowerConstants[0].kInverted = true;
        }

        public static PositionSubsystemConstants kClimbWinchConstants = new PositionSubsystemConstants();

        static {
            // Name of the subsystem, for example "Launcher Flywheels"
            kClimbWinchConstants.kSubsystemName = "Climb Winch"; 

            // Name of the subsystem, for example "Launcher"
            kClimbWinchConstants.kSuperstructureName = "Climb";

            // An enum which is in the template subsystem
            kClimbWinchConstants.kSubsystemType = PositionSubsystemType.CLIMB_WINCH;

            // The main motor constants
            // Instantiate these motor constants above this static block
            kClimbWinchConstants.kLeaderConstants = kClimbWinchLeaderConstants;

            // An array of motor constants that follow the leader
            // Instantiate these motor constants above this static block
            kClimbWinchConstants.kFollowerConstants = kWristFollowerConstants;

            // Initial, Manual, and Transition state of the subsytem
            // This enum is in the Subsystem that extends the MultiMotorPositionSubsystem
            // You will have to create these states
            kClimbWinchConstants.kInitialState = ClimbWinchState.DOWN;
            kClimbWinchConstants.kManualState = ClimbWinchState.MANUAL;
            kClimbWinchConstants.kTransitionState = ClimbWinchState.TRANSITION;

            // Home position of the motor
            kClimbWinchConstants.kHomePosition = 0.0;

            // Conversion factor for the motor output units
            // To find degrees: 360/gear ratio ex 360/100 for 100:1
            // For example for ratio 100:1 do 100
            kClimbWinchConstants.kPositionConversionFactor = 1.0; 

            // Tolerance for atSetpoint()
            kClimbWinchConstants.kSetpointTolerance = 0.1; 

            // PID Slot, make more if more than one set of pid constants are used
            kClimbWinchConstants.kDefaultSlot = 0; 

            // Max velocity and acceleration for trapezoidal motion profile
            kClimbWinchConstants.kMaxVelocity = 10; 
            kClimbWinchConstants.kMaxAcceleration = 20;

            // Max/Min positions the subsystem should be able to move
            kClimbWinchConstants.kMaxPosition = Double.POSITIVE_INFINITY;
            kClimbWinchConstants.kMinPosition = Double.NEGATIVE_INFINITY;

            // Enum which is found in SubsystemConstants
            kClimbWinchConstants.kManualControlMode = ManualControlMode.TRIGGERS;

            // Multiplied by controller inputs
            kClimbWinchConstants.kManualMultiplier = 1;

            // Deadband for controller
            kClimbWinchConstants.kManualDeadBand = .1;
        }

    }

}
