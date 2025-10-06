package frc.robot.Subsystems.Dispenser;

import java.util.function.Supplier;

import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;

public class Dispenser extends SubsystemBase {
  private final double CORAL_POSITIONING_SPEED = -0.2;
  private final double CORAL_DISPENSING_SPEED = 1;
  private final double CORAL_PULL_BACK_SPEED = -0.2;

  private final boolean DEBUG_MODE = true;
  private final boolean IS_INVERTED = true;

  private static Dispenser m_instance = new Dispenser();

  private EverMotorController m_dispenserMotor;
  private Supplier<Boolean> m_isAtEntry, m_isAtExit;


  private Dispenser() {
    EverSparkMax motor = new EverSparkMax(14);
    motor.setInverted(IS_INVERTED);
    
    SparkMaxConfig config = new SparkMaxConfig();
    LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();

    limitSwitchConfig.forwardLimitSwitchEnabled(false);
    limitSwitchConfig.reverseLimitSwitchEnabled(false);
    //config.alternateEncoder.countsPerRevolution(1);
    config.apply(limitSwitchConfig);
    
    motor.getControllerInstance().configure(config, null, null);

    //m_isAtExit = () -> {return motor.getControllerInstance().getForwardLimitSwitch().isPressed();};
    m_isAtEntry = () -> {return  motor.getControllerInstance().getReverseLimitSwitch().isPressed();};
    m_isAtExit = () -> {return motor.getControllerInstance().getForwardLimitSwitch().isPressed();};

    m_dispenserMotor = motor;
  }

  public static Dispenser getInstance() {
    return m_instance;
  }

  public void dispenseCoral() {
    m_dispenserMotor.set(CORAL_DISPENSING_SPEED);
  }

  public void slowDispense(){
    m_dispenserMotor.set(0.12);
  }

  public void pullCoral(){
    m_dispenserMotor.set(CORAL_PULL_BACK_SPEED);
  }


  public void stop(){
    m_dispenserMotor.stop();
  }
  private static boolean turnOffPositioning = false;
  @Override 
  public void periodic() {
    if (DEBUG_MODE) 
      log();
    
    //supposed to hold the coral inside if it tries to escape 
    if(m_dispenserMotor.get() == 0 && (isAtExit()) && turnOffPositioning == false){
      m_dispenserMotor.set(CORAL_POSITIONING_SPEED);
      turnOffPositioning = true;
    }
    if(turnOffPositioning && !isAtExit()){
      turnOffPositioning = false;
      m_dispenserMotor.set(0);

    }
    
  
  } 

  private void log() {
    SmartDashboard.putBoolean("Entry Sensor", m_isAtEntry.get());
    SmartDashboard.putBoolean("Exit Sensor", m_isAtExit.get());
    SmartDashboard.putBoolean("Inverted", true);
    SmartDashboard.putString("Idle mode", "Brake");
  }

  public boolean isAtEntry(){
    return m_isAtEntry.get();
  }

  public boolean isAtExit(){
    return m_isAtExit.get();
  }

  public boolean areMotorControllersConnected(){
    return m_dispenserMotor.isConnected();
  }
}
