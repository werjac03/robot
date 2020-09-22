/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.*;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climb extends SubsystemBase {
  /**
   * Creates a new Climb.
   */
  // PID constants for both climb arms
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, rotations;

  // initialize motors
  private final CANSparkMax climbL, climbR;

  // Motor encoders
  private CANEncoder climbLEncoder, climbREncoder;

  // motor PIDControllers
  private CANPIDController climbLController, climbRController;

  public Climb() {
    // settings for left climb motor
    climbL = new CANSparkMax(Constants.ClimbConstants.motorL, MotorType.kBrushless);
    climbL.restoreFactoryDefaults();
    climbL.setIdleMode(IdleMode.kBrake);
    climbL.setSmartCurrentLimit(60);
    // climbL.enableSoftLimit(SoftLimitDirection.kForward, true);
    // climbL.setSoftLimit(SoftLimitDirection.kForward, 310);

    // climbL.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // climbL.setSoftLimit(SoftLimitDirection.kReverse, 0);

    climbL.burnFlash();
    climbLEncoder = climbL.getEncoder();
    climbLEncoder.setPosition(0);
    climbLController = climbL.getPIDController();

    // settings for right climb motor
    climbR = new CANSparkMax(Constants.ClimbConstants.motorR, MotorType.kBrushless);
    climbR.restoreFactoryDefaults();
    climbR.setIdleMode(IdleMode.kBrake);
    climbR.setSmartCurrentLimit(60);
    // climbR.enableSoftLimit(SoftLimitDirection.kForward, true);
    // climbR.setSoftLimit(SoftLimitDirection.kForward, 0);

    // climbR.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // climbR.setSoftLimit(SoftLimitDirection.kReverse, -310);

    climbR.burnFlash();
    climbREncoder = climbR.getEncoder();
    climbREncoder.setPosition(0);
    climbRController = climbR.getPIDController();

    // // PID coefficients
    kP = 0.25;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 1.0 / 5676.0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // left climb motor PID
    climbLController.setP(kP);
    climbLController.setI(kI);
    climbLController.setD(kD);
    climbLController.setIZone(kIz);
    climbLController.setFF(kFF);
    climbLController.setOutputRange(kMinOutput, kMaxOutput);

    // right climb motor PID
    climbRController.setP(kP);
    climbRController.setI(kI);
    climbRController.setD(kD);
    climbRController.setIZone(kIz);
    climbRController.setFF(kFF);
    climbRController.setOutputRange(kMinOutput, kMaxOutput);


    SmartDashboard.putNumber("Climb Position", rotations);
  }

  public void setPosition() {
  climbLController.setReference(-SmartDashboard.getNumber("Climb Position", 0), ControlType.kPosition);
  SmartDashboard.putNumber("LClimbEncoder", climbLEncoder.getPosition());

  climbRController.setReference(SmartDashboard.getNumber("Climb Position", 0), ControlType.kPosition);
  SmartDashboard.putNumber("RClimbEncoder", climbREncoder.getPosition());
  }

  public void stop() {
    climbL.set(0);
    climbR.set(0);
  }

  public void periodic() {
    if (Math.abs(RobotContainer.m_operatorController.getRawAxis(5)) > 0.2) {
      climbL.set(-RobotContainer.m_operatorController.getRawAxis(5));
    } else {
      climbL.set(0);
    }
    if (Math.abs(RobotContainer.m_operatorController.getRawAxis(1)) > 0.2) {
      climbR.set(RobotContainer.m_operatorController.getRawAxis(1));
    } else {
      climbR.set(0);
    }
    
  SmartDashboard.putNumber("LClimbEncoder", climbLEncoder.getPosition());
  SmartDashboard.putNumber("RClimbEncoder", climbREncoder.getPosition());
  }

}
