package frc.robot.Subsystems.Dispenser;

import java.util.function.Supplier;

import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;

public class Dispenser extends SubsystemBase {
  private final double CORAL_POSITIONING_SPEED = 0.1;
  private final boolean DEBUG_MODE = true;

  private static Dispenser m_instance = new Dispenser();

  private EverMotorController m_dispenserMotor;
  private Supplier<Boolean> m_isAtEntry, m_isAtExit;
  
  private boolean m_algaeDropDispenseMode;
  private boolean m_dispenseMode;

  private Dispenser() {
    EverSparkMax motor = new EverSparkMax(14);
    
    SparkMaxConfig config = new SparkMaxConfig();
    LimitSwitchConfig limitSwitchConfig = new LimitSwitchConfig();

    limitSwitchConfig.forwardLimitSwitchEnabled(false);
    limitSwitchConfig.reverseLimitSwitchEnabled(false);
    config.apply(limitSwitchConfig);
    motor.getControllerInstance().configure(config, null, null);

    m_isAtExit = () -> {return motor.getControllerInstance().getForwardLimitSwitch().isPressed();};
    m_isAtEntry = () -> {return motor.getControllerInstance().getReverseLimitSwitch().isPressed();};

    m_dispenserMotor = motor;

    m_algaeDropDispenseMode = false; 
    m_dispenseMode = false;
  }

  public static Dispenser getInstance() {
    return m_instance;
  }

  public void dispenseCoral() {
    m_dispenseMode = true;
  }

 

  public void dropAlgea() { 
    m_algaeDropDispenseMode = true;
  }

  public void stop(){
    m_dispenserMotor.stop();
    m_dispenseMode = false;
    m_algaeDropDispenseMode = false;
  }

  public boolean isCoralInside() { 
    return m_isAtExit.get() && m_isAtEntry.get(); 
  }

  public boolean isCoralReadyToIntake(){
    return !m_isAtExit.get() && m_isAtEntry.get();
  }

  public boolean isCoralAtAlgaeDropPosition(){
    return m_isAtExit.get() && !m_isAtEntry.get();
  }


  @Override 
  public void periodic() {
    if (DEBUG_MODE) 
      log();

    if((isCoralReadyToIntake() && !m_dispenseMode && !m_algaeDropDispenseMode) || (isCoralInside() && m_algaeDropDispenseMode)){
      m_dispenserMotor.set(CORAL_POSITIONING_SPEED);
    }
    if((isCoralInside() && !m_dispenseMode && !m_algaeDropDispenseMode) || (isCoralAtAlgaeDropPosition() && m_algaeDropDispenseMode)){
      stop();
    }
    if(m_dispenseMode){
      double dispenseSpeed = Elevator.getInstance().getTargetLevel().dispenseSpeed;
      m_dispenserMotor.set(dispenseSpeed);
    }
   
  
  } 

  private void log() {
    
      SmartDashboard.putBoolean("Entry Sensor", m_isAtEntry.get());
      SmartDashboard.putBoolean("Exit Sensor", m_isAtExit.get());
      SmartDashboard.putBoolean("Inverted", true);
      SmartDashboard.putString("Idle mode", "Brake");
      SmartDashboard.putBoolean("Has coral", isCoralInside());
    
  }

  public boolean isAtEntry(){
    return m_isAtEntry.get();
  }

}
