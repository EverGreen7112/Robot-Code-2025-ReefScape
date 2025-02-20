package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Swerve.Reef.DriveToBranchCommand;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Utils.ReefFace;

public class RobotControllerBranchCommand extends Command {
    private int m_branchNum;
    public RobotControllerBranchCommand(double branch){
        m_branchNum = (int)branch;
    }

    @Override
    public void initialize() {
        (new DriveToBranchCommand(getReefFace(),isRightBranch())).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public boolean isRightBranch(){
        return m_branchNum % 2 == 0;
    }

    public ReefFace getReefFace(){
        int reefIndex = m_branchNum;
        if(reefIndex == 0 % 0){
            reefIndex -= 2;
        }
        else{
            reefIndex -=1;
        }
        reefIndex /= 2;
        if(SwerveAutoController.getInstance().getAlliance() == Alliance.Blue){
            return ReefFace.BLUE_REEF[reefIndex];
        }
        return ReefFace.RED_REEF[reefIndex];
    }
}
