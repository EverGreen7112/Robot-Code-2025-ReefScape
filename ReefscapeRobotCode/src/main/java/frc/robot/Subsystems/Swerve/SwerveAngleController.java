package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.Math.Funcs;

public class SwerveAngleController implements Periodic{

    private ProfiledPIDController m_angleController;
    private double m_targetAngle;
    private boolean m_isFieldOriented;
    private static SwerveAngleController m_instance = new SwerveAngleController();

    private SwerveAngleController(){
        m_angleController = new ProfiledPIDController(4, 0, 0, new Constraints(180, 180));
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
        start(PeriodicTime.kAutonomousPeriodic, PeriodicTime.kTeleopPeriodic, PeriodicTime.kTestPeriodic);
    }

    public void start(double targetAngle, boolean isFieldOriented){
        stop();
        m_targetAngle = targetAngle;
        m_isFieldOriented = isFieldOriented;
        m_angleController.reset( (m_isFieldOriented) ? SwerveLocalizer.getInstance().getFieldOrientedAngle() : Swerve.getInstance().getGyroOrientedAngle());

        start(PeriodicTime.kAutonomousPeriodic, PeriodicTime.kTeleopPeriodic, PeriodicTime.kTestPeriodic);
    }

    @Override
    public void periodic() {
        double angularVelocity;
        if(m_isFieldOriented){
            double currentAngle = SwerveLocalizer.getInstance().getFieldOrientedAngle();
            double angleError = m_targetAngle - currentAngle;
            // ensure error is in the range [-180, 180]
            if (angleError > 180) {
                angleError -= 360;
            } else if (angleError < -180) {
                angleError += 360;
            }
            angularVelocity = m_angleController.calculate(0, angleError);
        }
        else{
            double currentAngle = Swerve.getInstance().getGyroOrientedAngle();
            angularVelocity = m_angleController.calculate(currentAngle, currentAngle + Funcs.getShortestAnglePath(currentAngle, m_targetAngle));

        }
        Swerve.getInstance().driveByAngularVelocity(angularVelocity);   

       
        

    }

    public void stop(){
        stop(PeriodicTime.kAutonomousPeriodic, PeriodicTime.kTeleopPeriodic, PeriodicTime.kTestPeriodic);
    }
    
}
