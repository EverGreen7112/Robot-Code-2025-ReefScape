package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Utils.EverKit.Periodic;

public class SwerveAngleController implements Periodic{

    private ProfiledPIDController m_angleController;
    private double m_targetAngle;
    private boolean m_isFieldOriented;
    private static SwerveAngleController m_instance = new SwerveAngleController();

    private SwerveAngleController(){
        m_angleController = new ProfiledPIDController(2, 0, 0, new Constraints(180, 180));
        m_angleController.enableContinuousInput(-180, 180);
        m_isFieldOriented = false;   
    }

    public static SwerveAngleController getInstance(){
        return m_instance;
    }

    public void start(double targetAngle){
        stop();
        m_targetAngle = targetAngle;
        m_isFieldOriented = false;
        m_angleController.reset(Swerve.getInstance().getGyroOrientedAngle());
    }

    public void start(double targetAngle, boolean isFieldOriented){
        stop();
        m_targetAngle = targetAngle;
        m_isFieldOriented = isFieldOriented;
        m_angleController.reset( (m_isFieldOriented) ? SwerveLocalizer.getInstance().getFieldOrientedAngle() : Swerve.getInstance().getGyroOrientedAngle());

    }

    @Override
    public void periodic() {
        double currentAngle = (m_isFieldOriented ) ? SwerveLocalizer.getInstance().getFieldOrientedAngle() : Swerve.getInstance().getGyroOrientedAngle();
        Swerve.getInstance().driveByAngularVelocity( m_angleController.calculate(currentAngle, m_targetAngle));    
    }

    public void stop(){
        stop(PeriodicTime.kAutonomousPeriodic, PeriodicTime.kTeleopPeriodic, PeriodicTime.kTestPeriodic);
    }

    public void initialize(){
        start(PeriodicTime.kAutonomousPeriodic, PeriodicTime.kTeleopPeriodic, PeriodicTime.kTestPeriodic);

    }
    
}
