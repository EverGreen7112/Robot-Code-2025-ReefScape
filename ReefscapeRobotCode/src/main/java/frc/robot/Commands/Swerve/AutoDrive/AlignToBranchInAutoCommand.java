package frc.robot.Commands.Swerve.AutoDrive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Utils.ReefFace;

public class AlignToBranchInAutoCommand extends Command{
    private int branchIdx;
    private boolean isRight;
    private Command command;

    public AlignToBranchInAutoCommand(int branchIdx, boolean isRight){
        addRequirements(Swerve.getInstance());
        this.branchIdx = branchIdx;
        this.isRight = isRight;
    }

    @Override
    public void initialize() {
        if(SwerveAutoController.getInstance().getAlliance() == Alliance.Blue){
            command = (new AlignToBranchCommand(ReefFace.BLUE_REEF[branchIdx], isRight));
        }else{
            command = (new AlignToBranchCommand(ReefFace.RED_REEF[branchIdx], isRight));
        }
        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
