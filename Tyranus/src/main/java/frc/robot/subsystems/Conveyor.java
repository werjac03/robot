package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Conveyor extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushed);
  private DigitalInput sensor = new DigitalInput(9);
  private boolean send = false;
    
  public Conveyor() {
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.burnFlash();

  }
  public void autoIndex() {
    if(!sensor.get())
      motor.set(-0.5);
    else
      motor.set(0);
  }

  public void manualControl(double speed) {
    motor.set(-speed);
  }
  public void feedShooter(double speed, boolean atSpeed) {
    if (atSpeed)
      motor.set(-speed);
    else
      motor.set(0);
  }
  public void full53ND(double speed, boolean atSpeed) {
    if (atSpeed)
      send = true;
    if (send)
      motor.set(-1);
    else
      motor.set(0);
  }

  public void setSend() {
    send = false;
  }
  @Override
  public void periodic() {
  }
}
