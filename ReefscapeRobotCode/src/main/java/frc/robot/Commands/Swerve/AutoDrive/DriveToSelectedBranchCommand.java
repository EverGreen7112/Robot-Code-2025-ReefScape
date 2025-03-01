package frc.robot.Commands.Swerve.AutoDrive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.RobotOperatorController;

public class DriveToSelectedBranchCommand extends Command {
    private int m_branchNum;
    private DriveToBranchCommand m_command;
    public DriveToSelectedBranchCommand(){
    }

    @Override
    public void initialize() {
        m_branchNum = (int)RobotOperatorController.getInstance().getBranch();
        m_command = new DriveToBranchCommand(getReefFace(),isRightBranch());
        m_command.schedule();
    }

    @Override
    public boolean isFinished() {
        return m_command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_command.cancel();
    }

    public boolean isRightBranch(){
        return m_branchNum % 2 == 0;
    }

    public ReefFace getReefFace(){
        int reefIndex = m_branchNum;
        if(reefIndex % 2 == 0){
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
