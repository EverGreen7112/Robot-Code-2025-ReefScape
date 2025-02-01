package frc.robot.Subsystems.Dispenser;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;

public class Dispenser extends SubsystemBase {
  
  private EverMotorController m_dispenserMotor;
  private static Dispenser m_instance = new Dispenser();
  private DigitalInput m_entrySensor, m_exitSensor;
  private boolean m_hasCoral; // for now only its corrent use is to check if the dispenser has coral in it
  private final boolean IS_INVERTED = false;

  private Dispenser() {
    EverSparkMax motor = new EverSparkMax(0);
    motor.setInverted(IS_INVERTED);
    motor.setIdleMode(IdleMode.kBrake);
    m_dispenserMotor = motor;
    
    m_entrySensor = new DigitalInput(0);
    m_exitSensor = new DigitalInput(1);
  }

  public static Dispenser getInstance() {
    return m_instance;
  }

  public void dispanseCoral() {
    m_dispenserMotor.set(1);
  }

  public void dropAlgea() { 
    if (m_exitSensor.get() && m_entrySensor.get())
      m_dispenserMotor.set(0.2);
  }

  public void getCoralInPosition(){
    if (m_entrySensor.get() && !m_exitSensor.get()) 
      m_dispenserMotor.set(0.1);
  }

  public void stop(){
    m_dispenserMotor.set(0);
  }

  public boolean hasCoral() {
    return m_hasCoral;
  }

  @Override 
  public void periodic() {
    getCoralInPosition();
    if (!m_exitSensor.get() && !m_entrySensor.get())
      m_hasCoral = false;

    else
      m_hasCoral = true;

    SmartDashboard.putBoolean("Has Coral", m_hasCoral);

  } 
}
