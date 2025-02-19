package frc.robot.Commands.Swerve.Reef;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.Math.Funcs;

public class DriveToBranchCommand extends Command {

    private final double ALIGNMENT_DIS = 0.2;

    private ReefFace m_reefFace;
    private boolean m_isRightBranch;
    private Command m_driveCommand;
    private Pose2d m_targetBranch;

    public DriveToBranchCommand(ReefFace reefFace, boolean isRightBranch) {
        m_reefFace = reefFace;
        m_isRightBranch = isRightBranch;
    }

    @Override
    public void initialize() {
        m_targetBranch = (m_isRightBranch) ? m_reefFace.getRightBranchRobotPose() : m_reefFace.getLeftBranchRobotPose();
        Pose2d currentPose = SwerveLocalizer.getInstance().getCurrentPoint();
        
        //use pathplanner only for long distances
        if(Funcs.getDis(currentPose, m_targetBranch) > ALIGNMENT_DIS){
            Pose2d beforeBranch = m_targetBranch.plus(new Transform2d(-ALIGNMENT_DIS, 0, new Rotation2d()));
            m_driveCommand = SwerveAutoController.getInstance().generateDriveToCommand(beforeBranch)
                             .andThen(new AlignToBranchCommand(m_reefFace, m_isRightBranch));
        }
        else{
            m_driveCommand = new AlignToBranchCommand(m_reefFace, m_isRightBranch);
        }

        m_driveCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return !m_driveCommand.isScheduled();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveCommand.cancel();
    }
}
