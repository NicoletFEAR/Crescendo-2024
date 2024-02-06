package frc.robot.subsystems.intake;

import frc.robot.subsystems.templates.SubsystemConstants.RevMotorType;
import frc.robot.subsystems.templates.SubsystemConstants.SparkConstants;
import frc.robot.subsystems.templates.SubsystemConstants.VoltageSubsystemConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.subsystems.templates.VoltageSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Hold extends VoltageSubsystem {

    private static Hold m_instance = null;

    protected Hold(VoltageSubsystemConstants constants) {
        super(constants);
        //TODO Auto-generated constructor stub
    }

    public static Hold getInstance() {
        if (m_instance == null) {
            m_instance = new Hold(holdConstants.kHoldConstants);
        }

        return m_instance;
    }

    @Override
    public void subsystemPeriodic() {
        // TODO Auto-generated method stub
        SmartDashboard.putNumber("Velocity of Hold", getVelocity());
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
    }

    public enum HoldState implements VoltageSubsystemState {
        OFF(0, "Off"),
        IN(5, "In");
        
        private double voltage;
        private String name;
    
        private HoldState(double voltage, String name) {
          this.voltage = voltage;
          this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }

        @Override
        public double getVoltage() {
            return voltage;
        }

        // @Override
        // public void setVoltage(double voltage) {
        //     this.voltage = voltage;
        // }
    }

    public class holdConstants {
        public static final SparkConstants kHoldLeaderConstants = new SparkConstants();

        static {
            kHoldLeaderConstants.kID = MotorConstants.kHoldID;
            kHoldLeaderConstants.kRevMotorType = RevMotorType.CAN_SPARK_MAX;
            kHoldLeaderConstants.kName = "Intake Flywheel";
            kHoldLeaderConstants.kIdleMode = IdleMode.kBrake;
            kHoldLeaderConstants.kMotorType = MotorType.kBrushless;
            kHoldLeaderConstants.kCurrentLimit = 80;
            kHoldLeaderConstants.kInverted = false;
            kHoldLeaderConstants.kKp = 0.00001;
            kHoldLeaderConstants.kKi = 0.0;
            kHoldLeaderConstants.kKd = 0.0;
            kHoldLeaderConstants.kKff = 0.0001675;
        }

        public static final VoltageSubsystemConstants kHoldConstants =
            new VoltageSubsystemConstants();

        static {
            kHoldConstants.kSubsystemName = "Intake Flywheel";
            kHoldConstants.kSuperstructureName = "Intake";

            kHoldConstants.kSubsystemType = VoltageSubsystemType.HOLD;

            kHoldConstants.kLeaderConstants = kHoldLeaderConstants;
           
            kHoldConstants.kInitialState = HoldState.OFF;
        }
    }

    
}