package frc.robot.Commands.Swerve.Reef;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;

public class DriveToBranchCommand extends Command {

    private ReefFace m_reefFace;
    private boolean m_isRightBranch;

    public DriveToBranchCommand(ReefFace reefFace, boolean isRightBranch) {
        addRequirements(Swerve.getInstance());
        m_reefFace = reefFace;
        m_isRightBranch = isRightBranch;
    }

    @Override
    public void initialize() {
        Pose2d targetBranch = (m_isRightBranch) ? m_reefFace.getRightBranchRobotPose() : m_reefFace.getLeftBranchRobotPose();

        
        SwerveAutoController.getInstance().generateDriveToCommand(
                new GoalEndState(0, m_reefFace.getFacePose().getRotation()),
                targetBranch.plus(new Transform2d(-0.1, 0, new Rotation2d())),
                targetBranch).andThen(new AlignToBranchCommand(m_reefFace, m_isRightBranch)).schedule();

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
