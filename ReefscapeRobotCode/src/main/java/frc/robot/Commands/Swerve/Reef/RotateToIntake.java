package frc.robot.Commands.Swerve.Reef;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveAngleController;
import frc.robot.Subsystems.Swerve.SwerveAutoController;

public class RotateToIntake extends Command{
    private boolean m_isRight;
    
    public RotateToIntake(boolean isRight){
        m_isRight = isRight;
    }

    @Override
    public void initialize() {
        if(m_isRight){    
            SwerveAngleController.getInstance().start((SwerveAutoController.getInstance().getAlliance() == Alliance.Blue ? 360 - 55: 55));
        }else{
            SwerveAngleController.getInstance().start((SwerveAutoController.getInstance().getAlliance() == Alliance.Blue ? 45: 360 - 55));
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
