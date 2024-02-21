package frc.robot.subsystems.launcher;


import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;
import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.templates.VelocitySubsystem;

public class LauncherFlywheel extends VelocitySubsystem {

    private static LauncherFlywheel m_instance = null;

    public LauncherFlywheel(VelocitySubsystemConstants constants) {
        super(constants);
    }

    public static LauncherFlywheel getInstance() {
        if (m_instance == null) {
            m_instance = new LauncherFlywheel(LauncherFlywheelConstants.kLauncherFlywheelConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
        // setFeedforward(m_flywheelFeedForward.calculate(getVelocity()));


        // LauncherFlywheelState.FIELD_BASED_VELOCITY.setVelocity(calculateRPM());
    }

    @Override
    public void outputTelemetry() {
        // SmartDashboard.putBoolean("Beam Break", m_beamBreak.get());
    }

    public double calculateRPM() {
        // double distance = SwerveDrive.getInstance().getPose().getTranslation().getDistance(DriveConstants.kBlueSpeakerPosition);

        // if (distance > 0 && distance < LauncherConstants.kDistanceRPMMap.lastKey()) {
        //     double lowerRPM = LauncherConstants.kDistanceRPMMap.get(LauncherConstants.kDistanceRPMMap.floorKey(distance));
        //     double upperRPM = LauncherConstants.kDistanceRPMMap.get(LauncherConstants.kDistanceRPMMap.ceilingKey(distance));
        //     return lowerRPM + (distance - Math.floor(distance)) * (upperRPM - lowerRPM);
        // } else {
        //     return 0;
        // }
        return 0;
    }

    public enum LauncherFlywheelState implements VelocitySubsystemState {
        OFF(new double[] {0, 0}, "Off"),
        IDLE(new double[] {2500, 2500}, "Idle"),
        RUNNING(new double[] {5000, 5000}, "Running"),
        TRANSITION(new double[] {0, 0}, "Transition"),
        FIELD_BASED_VELOCITY(new double[] {0, 0}, "Field Based Velocity"),
        INTAKING(new double[] {-500, -500}, "Intaking"),
        MANUAL(new double[] {0, 0}, "Manual");
    
        private double[] velocity;
        private String name;
    
        private LauncherFlywheelState(double[] velocity, String name) {
          this.velocity = velocity;
          this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }

        @Override
        public double[] getVelocity() {
            return velocity;
        }

        @Override
        public void setVelocity(double[] velocity) {
            this.velocity = velocity;
        }
    }
    
    public class LauncherFlywheelConstants {
        public static final SparkConstants kTopLauncherFlywheelConstants = new SparkConstants();

        static {
            kTopLauncherFlywheelConstants.kID = MotorConstants.kLauncherTopFlywheelID;
            kTopLauncherFlywheelConstants.kRevMotorType = RevMotorType.CAN_SPARK_FLEX;
            kTopLauncherFlywheelConstants.kName = "Top Launcher Flywheel";
            kTopLauncherFlywheelConstants.kIdleMode = IdleMode.kCoast;
            kTopLauncherFlywheelConstants.kMotorType = MotorType.kBrushless;
            kTopLauncherFlywheelConstants.kCurrentLimit = 80;
            kTopLauncherFlywheelConstants.kInverted = false;
            kTopLauncherFlywheelConstants.kKp = 0.00001;
            kTopLauncherFlywheelConstants.kKi = 0.0;
            kTopLauncherFlywheelConstants.kKd = 0.0;
            kTopLauncherFlywheelConstants.kKff = 0.0001675;
        }

        public static final SparkConstants kBottomLauncherFlywheelConstants = new SparkConstants();

        static {
            kBottomLauncherFlywheelConstants.kID = MotorConstants.kLauncherBottomFlywheelID;
            kBottomLauncherFlywheelConstants.kRevMotorType = RevMotorType.CAN_SPARK_FLEX;
            kBottomLauncherFlywheelConstants.kName = "Bottom Launcher Flywheel";
            kBottomLauncherFlywheelConstants.kIdleMode = IdleMode.kCoast;
            kBottomLauncherFlywheelConstants.kMotorType = MotorType.kBrushless;
            kBottomLauncherFlywheelConstants.kCurrentLimit = 80;
            kBottomLauncherFlywheelConstants.kInverted = false;
            kBottomLauncherFlywheelConstants.kKp = 0.00001;
            kBottomLauncherFlywheelConstants.kKi = 0.0;
            kBottomLauncherFlywheelConstants.kKd = 0.0;
            kBottomLauncherFlywheelConstants.kKff = 0.0001675;
        }

        public static final VelocitySubsystemConstants kLauncherFlywheelConstants =
            new VelocitySubsystemConstants();

        static {
            kLauncherFlywheelConstants.kSubsystemName = "Launcher Flywheel";
            kLauncherFlywheelConstants.kSuperstructureName = "Launcher";

            kLauncherFlywheelConstants.kSubsystemType = VelocitySubsystemType.LAUNCHER_FLYWHEEL;

            kLauncherFlywheelConstants.kMotorConstants = new SparkConstants[] {kTopLauncherFlywheelConstants, kBottomLauncherFlywheelConstants};

            kLauncherFlywheelConstants.kVelocityConversionFactor = 1.0; 

            kLauncherFlywheelConstants.kDefaultSlot = 0;

            kLauncherFlywheelConstants.kSetpointTolerance = 3000;

            kLauncherFlywheelConstants.kInitialState = LauncherFlywheelState.OFF;
            kLauncherFlywheelConstants.kTransitionState = LauncherFlywheelState.TRANSITION;
            kLauncherFlywheelConstants.kManualState = LauncherFlywheelState.MANUAL;
        }
    }

}
