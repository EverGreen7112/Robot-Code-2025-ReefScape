package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverMotionMagicPIDController;

public class Elevator extends SubsystemBase {

    private static final boolean DEBUG_MODE = false;
    private final double ELEVATOR_TOLERANCE = 1;

    public enum ElevatorLevel{
        CLOSED(3, 0.3),
        L1(7, 0.3),
        L2(16.5, 0.3),
        L3(34.2, 0.3),
        L4(60.55,0.3); 

        public final double height;
        public final double dispenseSpeed;

        private ElevatorLevel(double height,double dispenseSpeed){
            this.height = height;
            this.dispenseSpeed = dispenseSpeed;
        }

    }

    private static Elevator m_instance = new Elevator();

    private ElevatorLevel m_targetLevel;
    private EverMotorController m_motor;
    private EverPIDController m_pidController;
    private EverEncoder m_encoder;

    private DigitalInput m_bottomLS;

    private Elevator(){
        m_targetLevel = ElevatorLevel.CLOSED;

        EverTalonFX talon = new EverTalonFX(14);
        talon.setIdleMode(IdleMode.kCoast);
        talon.getControllerInstance().setNeutralMode(NeutralModeValue.Brake);

        EverMotionMagicPIDController talonPidController = new EverMotionMagicPIDController(talon, 300, 120);
        Slot0Configs a = new Slot0Configs();
        a.kD = 0;
        a.kG = 0.4;
        a.kI = 0;
        a.kP = 3.2;
        a.kV = 1/2.6;
        a.kS = 0.2;
        a.GravityType = GravityTypeValue.Elevator_Static;
        talonPidController.setPID(a);

        EverTalonFXInternalEncoder encoder = new EverTalonFXInternalEncoder(talon);
        encoder.setPosConversionFactor(1);
        
        m_bottomLS = new DigitalInput(0);

        m_motor = talon;
        m_pidController = talonPidController;
        m_encoder = encoder;
        
    }

    public static Elevator getInstance(){
        return m_instance;
    }

    public double getPose(){
        return m_encoder.getPos();
    }

    public void moveManually(double output){
        m_motor.set(output);
    }

    public void moveToDesiredLevel(ElevatorLevel desiredLevel){
        m_targetLevel = desiredLevel;
        m_pidController.activate(desiredLevel.height, ControlType.kPos);
    }

    public ElevatorLevel getTargetLevel(){
        return m_targetLevel;
    }
    public boolean cantGoDown(){
        return false;
    }

    public boolean isOpenAt(ElevatorLevel level){
        return MathUtil.isNear(level.height, getPose(), ELEVATOR_TOLERANCE);
    }

    public void resetPose(){
        m_encoder.setPos(0);
    }

    @Override
    public void periodic() {
        if(DEBUG_MODE)
            log();

        if(cantGoDown() && m_motor.get() <= 0){
            m_motor.stop();
            resetPose();
        }
    }


    private void log(){
        SmartDashboard.putBoolean("bottom ls", !m_bottomLS.get());
        SmartDashboard.putNumber("motor output", m_motor.get());
        SmartDashboard.putNumber("height",m_encoder.getPos());
    }

    public boolean areMotorControllersConnected(){
        return m_motor.isConnected();
    }
  
}
