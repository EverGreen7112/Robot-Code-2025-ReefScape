package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.Dispenser.DispenseCoralCommand;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Elevator.ElevatorLevel;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Utils.Math.Vector2d;

public class BadAuto extends Command{
    @Override
    public void initialize() {
        new SequentialCommandGroup(
            new InstantCommand(() -> {Swerve.getInstance().drive(new Vector2d(1, 0), false, 0);
                Elevator.getInstance().moveToDesiredLevel(ElevatorLevel.L4);}),
            new WaitCommand(5)
            ,new InstantCommand(() -> {Swerve.getInstance().stop();})
            ,new DispenseCoralCommand()
        ).schedule();
    }
}
