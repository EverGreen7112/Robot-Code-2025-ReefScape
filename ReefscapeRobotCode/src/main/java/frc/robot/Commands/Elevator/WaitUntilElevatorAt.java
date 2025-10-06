package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorLevel;

public class WaitUntilElevatorAt extends Command{
    private ElevatorLevel m_level;

    public WaitUntilElevatorAt(ElevatorLevel level){
        m_level = level;

    }
     @Override
    public void initialize() {
        
    }


    @Override
    public boolean isFinished() {
        return Elevator.getInstance().isOpenAt(m_level);
    }
}
