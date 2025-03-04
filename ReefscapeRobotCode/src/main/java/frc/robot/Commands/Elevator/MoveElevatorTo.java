package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorLevel;

public class MoveElevatorTo extends Command {
    
    private ElevatorLevel m_targetLevel;

    public MoveElevatorTo(ElevatorLevel level){
        m_targetLevel = level;
    }
    

    @Override
    public void initialize(){
        Elevator.getInstance().moveToDesiredLevel(m_targetLevel);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
