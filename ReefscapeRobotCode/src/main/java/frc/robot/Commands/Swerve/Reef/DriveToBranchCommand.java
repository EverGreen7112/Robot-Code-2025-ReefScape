package frc.robot.Commands.Swerve.Reef;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Utils.ReefFace;

public class DriveToBranchCommand extends Command {

    private ReefFace m_reefFace;
    private boolean m_isRightBranch;
    private Command m_driveCommand;
    private BooleanSupplier m_stopCommand;
    private Pose2d m_targetBranch;

    public DriveToBranchCommand(ReefFace reefFace, boolean isRightBranch, BooleanSupplier stopCommand) {
        m_reefFace = reefFace;
        m_isRightBranch = isRightBranch;
        m_stopCommand = stopCommand;
    }

    @Override
    public void initialize() {
        m_targetBranch = (m_isRightBranch) ? m_reefFace.getRightBranchRobotPose() : m_reefFace.getLeftBranchRobotPose();
        m_driveCommand = SwerveAutoController.getInstance().generateDriveToCommand(
            m_targetBranch).andThen(new AlignToBranchCommand(m_reefFace, m_isRightBranch, m_stopCommand));
        m_driveCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return !m_driveCommand.isScheduled() || m_stopCommand.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        m_driveCommand.cancel();
    }
}
