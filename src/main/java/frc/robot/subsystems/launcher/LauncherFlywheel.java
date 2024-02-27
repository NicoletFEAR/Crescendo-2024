package frc.robot.subsystems.launcher;


import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;
import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.launcher.LauncherSuperstructure.LauncherConstants;
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


        LauncherFlywheelState.FIELD_BASED_VELOCITY.setVelocity(new double[] {calculateRPM(), calculateRPM()});
        // SmartDashboard.putNumber("Top Encoder Speed", m_encoders[0].getVelocity());
        // SmartDashboard.putNumber("Bottom Encoder Speed", m_encoders[1].getVelocity());
        // SmartDashboard.putNumber("Intended Speed", m_desiredState.getVelocity()[0]);
    }

    @Override
    public void outputTelemetry() {
        // SmartDashboard.putBoolean("Beam Break", m_beamBreak.get());
    }

    public double calculateRPM() {
        double distance;

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            distance = RobotContainer.m_drivebase.getPose().getTranslation().getDistance(DriveConstants.kRedSpeakerPosition);
        } else {
            distance = RobotContainer.m_drivebase.getPose().getTranslation().getDistance(DriveConstants.kBlueSpeakerPosition);
        }
        
        if (distance > 0 && distance < LauncherConstants.kDistanceRPMMap.lastKey()) {
            double lowerRPM = LauncherConstants.kDistanceRPMMap.get(LauncherConstants.kDistanceRPMMap.floorKey(distance));
            double upperRPM = LauncherConstants.kDistanceRPMMap.get(LauncherConstants.kDistanceRPMMap.ceilingKey(distance));
            return lowerRPM + (distance - Math.floor(distance)) * (upperRPM - lowerRPM);
        } else {
            return 0;
        }
    }

    public enum LauncherFlywheelState implements VelocitySubsystemState {
        OFF(new double[] {0, 0}, "Off"),
        IDLE(new double[] {0, 0}, "Idle"),
        WING_NOTE_1(new double[] {5000, 5000}, "Wing Note 1"),
        WING_NOTE_2(new double[] {5000, 5000}, "Wing Note 2"),
        WING_NOTE_3(new double[] {5000, 5000}, "Wing Note 3"),
        ADJUST_NOTE_IN(new double[] {-200, -200}, "Adjust Note In"),
        ADJUST_NOTE_OUT(new double[] {200, 200}, "Adjust Note Out"),
        RUNNING(new double[] {2350, 2350}, "Running"), // arbitrary testing value
        SUBWOOFER(new double[] {5000, 5000}, "Subwoofer"), // used for when against the base of the speaker
        PODIUM(new double[] {5000, 5000}, "Podium"), // used for when against the base of the PODIUM
        TRANSITION(new double[] {0, 0}, "Transition"),
        FIELD_BASED_VELOCITY(new double[] {0, 0}, "Field Based Velocity"),
        INTAKING(new double[] {-500, -500}, "Intaking"), // used when intaking through launch
        MANUAL(new double[] {0, 0}, "Manual");

        // see if slowing down the bottom flyhweeel helps with tubmle
    
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
            kTopLauncherFlywheelConstants.kKp = 0.00025;
            kTopLauncherFlywheelConstants.kKi = 0.0;
            kTopLauncherFlywheelConstants.kKd = 0.0;
            kTopLauncherFlywheelConstants.kKff = 0.00015;
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
            kBottomLauncherFlywheelConstants.kKp = 0.00025;
            kBottomLauncherFlywheelConstants.kKi = 0.0;
            kBottomLauncherFlywheelConstants.kKd = 0.0;
            kBottomLauncherFlywheelConstants.kKff = 0.00015;
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

            kLauncherFlywheelConstants.kSetpointTolerance = 200;

            kLauncherFlywheelConstants.kInitialState = LauncherFlywheelState.OFF;
            kLauncherFlywheelConstants.kTransitionState = LauncherFlywheelState.TRANSITION;
            kLauncherFlywheelConstants.kManualState = LauncherFlywheelState.MANUAL;
        }
    }

}
