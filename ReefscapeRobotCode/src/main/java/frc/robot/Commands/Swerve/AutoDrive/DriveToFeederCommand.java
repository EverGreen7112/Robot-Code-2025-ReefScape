package frc.robot.Commands.Swerve.AutoDrive;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.Math.Funcs;

public class DriveToFeederCommand extends Command {
    
    private boolean m_isRight, m_isInner;
    private Command m_driveCommand;
    private Pose2d[][] m_coralStationsBlue = //  <--------------------------- inner  side ------------------------>    <------------------------------ outer ----------------------------->
                                        {{new Pose2d(0.791,1.274, new Rotation2d(Math.toRadians(60))), new Pose2d(1.486,0.818, new Rotation2d(Math.toRadians(60)))}, // right
                                        {new Pose2d(0.791,6.680, new Rotation2d(Math.toRadians(300))), new Pose2d(1.486,7.256, new Rotation2d(Math.toRadians(300)))}}; // left

    private Pose2d[][] m_coralStationsRed = //  <--------------------------- inner  side ------------------------>    <------------------------------ outer ----------------------------->
                                            {{new Pose2d(16.831,6.680, new Rotation2d(Math.toRadians(240))), new Pose2d(15.968,7.256, new Rotation2d(Math.toRadians(240)))}, // right
                                            {new Pose2d(16.831,1.274, new Rotation2d(Math.toRadians(120))), new Pose2d(15.968,0.806, new Rotation2d(Math.toRadians(120)))}}; // left

    public DriveToFeederCommand(boolean isRight, boolean isInner) {
        this.m_isRight = isRight;
        this.m_isInner = isInner;
    }

    @Override
    public void initialize() {
        
        int row = (m_isInner) ? (0) : (1);
        int colomm = (m_isRight) ? (0) : (1);
        
        Pose2d coralStation = (SwerveAutoController.getInstance().getAlliance() == Alliance.Blue ? m_coralStationsBlue[row][colomm] : m_coralStationsRed[row][colomm]);

        m_driveCommand = SwerveAutoController.getInstance().generateDriveToCommand(coralStation);

        m_driveCommand.schedule();
    }

    @Override
    public void execute() {
        SwerveAutoController.isRobotAligning = true;
    }

    @Override
    public boolean isFinished() {
        return !m_driveCommand.isScheduled();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveAutoController.isRobotAligning = false;

        m_driveCommand.cancel();
    }
}
