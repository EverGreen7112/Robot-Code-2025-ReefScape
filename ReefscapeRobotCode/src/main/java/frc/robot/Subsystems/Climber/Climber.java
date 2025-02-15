package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Climber extends SubsystemBase{
    private final double CLIMB_SPEED = 0.3;
    private final boolean DEBUG_MODE = false;

    private static Climber m_instance = new Climber();

    private EverMotorController m_climbMotor;
    private DigitalInput m_bottomLS;
    
    private Climber(){
        m_climbMotor = new EverTalonFX(12);
        m_bottomLS = new DigitalInput(3);

    }

    public static Climber getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        if(cantOpen() && m_climbMotor.get() > 0)
            stop();

        if(DEBUG_MODE)
            log();

    }

    public void open(){
        m_climbMotor.set(CLIMB_SPEED);
    }

    public void close(){
        m_climbMotor.set(-CLIMB_SPEED);
    }

    public void stop(){
        m_climbMotor.stop();
    }

    private boolean cantOpen(){
        return m_bottomLS.get();
    }

    private void log(){
        SmartDashboard.putBoolean("bottom limit switch", cantOpen());
        SmartDashboard.putNumber("climber", m_climbMotor.get());
    }

    

    
    
}
