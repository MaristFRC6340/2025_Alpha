package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoScore extends SequentialCommandGroup{
    private SwerveSubsystem swerve;
    private ElevatorSubsystem elevator;
    private CoralSubsystem coral;
    private int reefPosition;
    private boolean left;
    

    public AutoScore(SwerveSubsystem swerve,  ElevatorSubsystem elevator, CoralSubsystem coral, int reefPosition, boolean left, boolean right) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.coral = coral;
        this.reefPosition = reefPosition;
        this.left = left;

        addCommands(swerve.getDriveToReefCommand(reefPosition, left),
        elevator.getSetPositionCommand(), // get int position later
        coral.getOuttakeCommand());
        
    }

    


}
