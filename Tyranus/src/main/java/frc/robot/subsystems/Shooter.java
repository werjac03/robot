/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooter1, shooter2;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setPoint, speed;

  public Shooter() {
    shooter1 = new CANSparkMax(12, MotorType.kBrushless);
    shooter2 = new CANSparkMax(13, MotorType.kBrushless);
    shooter1.restoreFactoryDefaults();
    shooter1.setIdleMode(IdleMode.kCoast);
    shooter1.setSmartCurrentLimit(60);
    shooter1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    shooter1.burnFlash();
    shooter2.restoreFactoryDefaults();
    shooter2.setIdleMode(IdleMode.kCoast);
    shooter2.setSmartCurrentLimit(60);
    shooter2.follow(shooter1, true);
    shooter2.burnFlash();

    m_encoder = shooter1.getEncoder();
    m_pidController = shooter1.getPIDController();
    kP = 30;
    kI = 0;
    kD = 0; 
    // kDf = 0.5;
    kIz = 0; 
    kFF = 1.0/5676.0; 
    kMaxOutput = 1; 
    kMinOutput = 0.8;
    maxRPM = 5676.0;
    speed = 4550.0;
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    SmartDashboard.putNumber("Shooter P", kP);
    SmartDashboard.putNumber("Shooter Velocity", speed);
  }

  public void runShooter() {
    setPoint = SmartDashboard.getNumber("Shooter Velocity", 4550);

    m_pidController.setReference(setPoint, ControlType.kVelocity);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("OutputCurrent", shooter1.get());
    
  }

  public void full53ND() {
    // setPoint = SmartDashboard.getNumber("Shooter HighSpeed", 5000);

    m_pidController.setReference(maxRPM, ControlType.kVelocity);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("OutputCurrent", shooter1.get());
    
  }
  public void setShooter() {
    shooter1.set(-0.5);
  }
  public void stopShooter() {
    shooter1.set(0);
  }
  public void periodic() {
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
  }
  public boolean atSpeed() {
    return (Math.abs(setPoint - m_encoder.getVelocity()))/(setPoint) < 0.05;
  }
}
