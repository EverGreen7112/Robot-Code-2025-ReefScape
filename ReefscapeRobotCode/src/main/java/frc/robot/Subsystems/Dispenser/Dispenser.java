package frc.robot.Subsystems.Dispenser;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;

public class Dispenser extends SubsystemBase {
  private final double DISPENSER_SPEED = 0.6, CORAL_POSITIONING_SPEED = 0.1;
  private final boolean DEBUG_MODE = true;


  private static Dispenser m_instance = new Dispenser();

  private EverMotorController m_dispenserMotor;
  private DigitalInput m_entryLS, m_exitLS;
  private boolean m_hasCoral; // for now only its corrent use is to check if the dispenser has coral in it

  private Dispenser() {
    EverSparkMax motor = new EverSparkMax(0);
    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kBrake);
    m_dispenserMotor = motor;
    
    m_entryLS = new DigitalInput(0);
    m_exitLS = new DigitalInput(1);
  }

  public static Dispenser getInstance() {
    return m_instance;
  }

  public void dispenseCoral() {
    m_dispenserMotor.set(DISPENSER_SPEED);
  }

  public void dropAlgea() { 
    if (getExitLS() && !getEntryLS())
      stop();
    dispenseCoral();
  }

  public void getCoralInPosition(){
    if (getEntryLS() && !getExitLS()) 
      m_dispenserMotor.set(CORAL_POSITIONING_SPEED);
  }

  public void stop(){
    m_dispenserMotor.stop();
  }

  public boolean hasCoral() {
    m_hasCoral = true;
    if ()
      m_hasCoral = !(!getExitLS() && !getEntryLS());
      
    return m_hasCoral;
  }

  public boolean getEntryLS() {
    return m_entryLS.get();
  }

  public boolean getExitLS() {
    return m_exitLS.get();
  }

  @Override 
  public void periodic() {
    if (DEBUG_MODE) 
      log();
    
    getCoralInPosition();
  } 

  private void log() {
    
      SmartDashboard.putBoolean("Entry Sensor", getEntryLS());
      SmartDashboard.putBoolean("Exit Sensor", getExitLS());
      SmartDashboard.putBoolean("Inverted", true);
      SmartDashboard.putString("Idle mode", "Brake");
      SmartDashboard.putNumber("Exit Sensor id", m_exitLS.getChannel());
      SmartDashboard.putNumber("Entry Sensor id", m_exitLS.getChannel());
      SmartDashboard.putBoolean("Has coral", hasCoral());
    
  }

}
