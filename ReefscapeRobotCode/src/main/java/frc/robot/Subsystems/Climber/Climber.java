package frc.robot.Subsystems.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class Climber extends SubsystemBase{
    private final double CLIMB_SPEED = 0.3;
    private final boolean DEBUG_MODE = true;

    private static Climber m_instance = new Climber();

    private EverMotorController m_climbMotor;
    public DigitalInput m_leftLS;
    public DigitalInput m_rightLS;
    private EverEncoder m_encoder;
    
    private Climber(){
        EverTalonFX climbMotor = new EverTalonFX(12);
        m_climbMotor = climbMotor;
        m_leftLS = new DigitalInput(8);
        m_rightLS = new DigitalInput(9);
        EverTalonFXInternalEncoder encoder = new EverTalonFXInternalEncoder(climbMotor);
        encoder.setPosConversionFactor(1);

        m_encoder = encoder;

    }

    public static Climber getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        if(DEBUG_MODE)
            log();
      
    }


    public boolean isCageLocked(){
        return !m_leftLS.get() && !m_rightLS.get();
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

    public boolean cantOpen(){
        return m_encoder.getPos() >= 300;//185;
        
    }

    

    private void log(){
        SmartDashboard.putBoolean("bottom limit switch", cantOpen());
        SmartDashboard.putNumber("climber", m_climbMotor.get());
        SmartDashboard.putNumber("climbe encoder", m_encoder.getPos());
        SmartDashboard.putBoolean("is cage locked", isCageLocked());
    }

    

    
    
}
