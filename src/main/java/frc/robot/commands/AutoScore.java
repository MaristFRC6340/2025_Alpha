package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
public class AutoScore extends SequentialCommandGroup{
    public AutoScore(SwerveSubsystem swerve,  ElevatorSubsystem elevator, CoralSubsystem coral, int reefPosition, boolean left, boolean right) {
        addCommands(swerve.getDriveToReefCommand(reefPosition, left),
        //add coral outtake
        coral.getOuttakeCommand());
    }
}
