package frc.robot.Subsystems.Dispenser;

import java.util.function.Supplier;

import javax.net.ssl.HandshakeCompletedListener;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;

public class Dispenser extends SubsystemBase {
  private final double DISPENSER_SPEED = 0.6, CORAL_POSITIONING_SPEED = 0.1;
  private final boolean DEBUG_MODE = true;


  private static Dispenser m_instance = new Dispenser();

  private EverMotorController m_dispenserMotor;
  private Supplier <Boolean> m_isAtEntry, m_isAtExit;
  private boolean m_firstTime = true;
  private SparkMaxConfig m_config = new SparkMaxConfig();
  private LimitSwitchConfig m_LimitSwitchConfig = new LimitSwitchConfig();
  private EverEncoder m_encoder;
  private double m_coralRestingPosition;
  private EverPIDController m_pidController;


  private Dispenser() {
    EverSparkMax motor = new EverSparkMax(0);
    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kCoast);
    m_dispenserMotor = motor;
    m_LimitSwitchConfig.forwardLimitSwitchEnabled(false);
    m_LimitSwitchConfig.reverseLimitSwitchEnabled(false);
    m_config.apply(m_LimitSwitchConfig);
    motor.getControllerInstance().configure(m_config, null, null);
    m_isAtEntry = () -> {return motor.getControllerInstance().getForwardLimitSwitch().isPressed();};
    m_isAtExit = () -> {return motor.getControllerInstance().getReverseLimitSwitch().isPressed();};
    m_encoder = new EverSparkInternalEncoder(motor);
    m_pidController = new EverSparkMaxPIDController(motor);

  }

  public static Dispenser getInstance() {
    return m_instance;
  }

  public void dispenseCoral() {
    m_dispenserMotor.set(DISPENSER_SPEED);
  }

  public void dropAlgea() { 
    if (m_isAtExit.get() && !m_isAtEntry.get())
      stop();
    dispenseCoral();
  }

  public void getCoralInPosition(){
    if (m_isAtEntry.get() && !m_isAtExit.get())  
      m_dispenserMotor.set(CORAL_POSITIONING_SPEED);
  }

  public double getCoralRestingPosition() {
    if ( m_isAtEntry.get() && m_isAtExit.get() && m_firstTime) {
      m_coralRestingPosition = m_encoder.getPos();
      m_firstTime = false;
      return  m_coralRestingPosition;
    }

    if (!m_isAtEntry.get() && !m_isAtExit.get())
      m_firstTime = true;
    
    return m_coralRestingPosition;

  }

  public void stop(){
    m_dispenserMotor.stop();
  }

  public boolean isCoralInPosition() { 
    return m_isAtExit.get() && m_isAtEntry.get(); 
  }

  @Override 
  public void periodic() {
    if (DEBUG_MODE) 
      log();
    
    getCoralInPosition();
    if (isCoralInPosition()) {
      m_pidController.activate(getCoralRestingPosition(), ControlType.kPos);
    }

  } 

  private void log() {
    
      SmartDashboard.putBoolean("Entry Sensor", m_isAtEntry.get());
      SmartDashboard.putBoolean("Exit Sensor", m_isAtExit.get());
      SmartDashboard.putBoolean("Inverted", true);
      SmartDashboard.putString("Idle mode", "Brake");
      SmartDashboard.putBoolean("Has coral", isCoralInPosition());
    
  }

}
