package frc.robot.subsystems.intake;

import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;
import frc.robot.subsystems.templates.SubsystemConstants.VelocitySubsystemConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.templates.VelocitySubsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeFlywheel extends VelocitySubsystem {

    private static IntakeFlywheel m_instance = null;
  

    protected IntakeFlywheel(VelocitySubsystemConstants constants) {
        super(constants);
    }

    public static IntakeFlywheel getInstance() {
        if (m_instance == null) {
            m_instance = new IntakeFlywheel(IntakeFlywheelConstants.kIntakeFlywheelConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {}

    @Override
    public void outputTelemetry() {}

    public enum IntakeFlywheelState implements VelocitySubsystemState {
        OFF(new double[] {0, 0}, "Off"),
        INTAKING(new double[] {-300, -300}, "Intaking"), // -6
        INTAKE_TO_LAUNCH(new double[] {-150, -150}, "Intake To Launch"), // -3.5
        EJECTING(new double[] {250, 250}, "Out"), // 5
        LAUNCH_TO_INTAKE(new double[] {50, 50}, "Out"), // 1.5
        AMP(new double[] {300, 300}, "Amp"), // 6
        TRANSITION(new double[] {0, 0}, "Transition"),
        MANUAL(new double[] {0, 0}, "Manual");
        
        private double[] velocity;
        private String name;
    
        private IntakeFlywheelState(double[] velocity, String name) {
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

    

    public class IntakeFlywheelConstants {
        public static final SparkConstants kIntakeFlywheelLeaderConstants = new SparkConstants();

        static {
            kIntakeFlywheelLeaderConstants.kID = MotorConstants.kIntakeFlywheelID;
            kIntakeFlywheelLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kIntakeFlywheelLeaderConstants.kName = "Intake Flywheel";
            kIntakeFlywheelLeaderConstants.kIdleMode = IdleMode.kBrake;
            kIntakeFlywheelLeaderConstants.kMotorType = MotorType.kBrushless;
            kIntakeFlywheelLeaderConstants.kCurrentLimit = 80;
            kIntakeFlywheelLeaderConstants.kInverted = false;
            kIntakeFlywheelLeaderConstants.kKp = 0.01; // 0.00001
            kIntakeFlywheelLeaderConstants.kKi = 0.0;
            kIntakeFlywheelLeaderConstants.kKd = 0.0;
            kIntakeFlywheelLeaderConstants.kKff = 0.0001675;
        }

        public static final VelocitySubsystemConstants kIntakeFlywheelConstants =
            new VelocitySubsystemConstants();

        static {
            kIntakeFlywheelConstants.kSubsystemName = "Intake Flywheel";
            kIntakeFlywheelConstants.kSuperstructureName = "Intake";

            kIntakeFlywheelConstants.kSubsystemType = VelocitySubsystemType.INTAKE_FLYWHEELS;

            kIntakeFlywheelConstants.kMotorConstants = new SparkConstants[] {kIntakeFlywheelLeaderConstants};
           
            kIntakeFlywheelConstants.kInitialState = IntakeFlywheelState.OFF;
        }
    }

    
}
