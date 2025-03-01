package frc.robot.Commands.Swerve.AutoDrive;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.Math.Funcs;

public class DriveToClosestBranchCommand extends Command {

    private boolean m_isRightBranch;
    private Command m_driveCommand;

    public DriveToClosestBranchCommand(boolean isRightBranch) {
        m_isRightBranch = isRightBranch;
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Drive to closest branch finshed", false);
        ReefFace[] reef = (SwerveAutoController.getInstance().getAlliance() == Alliance.Blue ? ReefFace.BLUE_REEF : ReefFace.RED_REEF);
        Pose2d currentPoint = SwerveLocalizer.getInstance().getCurrentPoint();
        double minDis = Funcs.getDis(currentPoint, reef[0].getFacePose());

        ReefFace closestFace = reef[0];
        for (int i = 1; i < 6; i++) {
            double currentDistance = Funcs.getDis(currentPoint, reef[i].getFacePose());
            if (minDis > currentDistance) {
                minDis = currentDistance;
                closestFace = reef[i];
            }
        }
        m_driveCommand = (new DriveToBranchCommand(closestFace, m_isRightBranch));
        m_driveCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return !m_driveCommand.isScheduled();
    }


    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Drive to closest branch finshed", true);
        m_driveCommand.cancel();
    }

    

}
