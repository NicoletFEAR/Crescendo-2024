// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.LauncherConstants;

// public class Wrist extends SubsystemBase {

//   private CANSparkMax m_wristMotor;
//   private RelativeEncoder m_wristEncoder;
//   private SparkPIDController m_controller;

//   private double intentedPosition;

//   /** Creates a new Wrist. */
//   public Wrist() {
//     m_wristMotor = new CANSparkMax(15, MotorType.kBrushless);

//     m_wristEncoder = m_wristMotor.getEncoder();

//     m_controller = m_wristMotor.getPIDController();

//     m_controller.setP(LauncherConstants.kp);
//     m_controller.setI(LauncherConstants.ki);
//     m_controller.setD(LauncherConstants.kd);

//     SmartDashboard.putNumber("Wrist Position", 0);
//     SmartDashboard.putNumber("Wrist Intended Position", 0);
//     SmartDashboard.putNumber("Wrist Error", 0);

//     m_wristEncoder.setPosition(0);
    
//   }

//   public void manualControl() {
//     double speed = RobotContainer.m_driverController.getRightTriggerAxis() - RobotContainer.m_driverController.getLeftTriggerAxis();

//     intentedPosition += (speed * .25);

//     intentedPosition = MathUtil.clamp(intentedPosition, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
//   }

//   @Override
//   public void periodic() {

//     SmartDashboard.putNumber("Wrist Position", m_wristEncoder.getPosition());
//     SmartDashboard.putNumber("Wrist Intended Position", intentedPosition);
//     SmartDashboard.putNumber("Wrist Error", Math.abs(m_wristEncoder.getPosition()-intentedPosition));
//     SmartDashboard.putNumber("Wrist Motor Voltage", m_wristMotor.getOutputCurrent());
//     SmartDashboard.putNumber("Wrist Motor Temp", m_wristMotor.getMotorTemperature());
    
//     m_controller.setReference(intentedPosition, ControlType.kPosition);
//     // This method will be called once per scheduler run
//   }
// }
