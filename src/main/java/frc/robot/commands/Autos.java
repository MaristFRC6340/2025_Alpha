package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
    
    public static Command getTwoCoral(Command autoAlignCommand, Command scoreCommand) {
        PathPlannerPath r21toCoralL = null;
        PathPlannerPath coralLToR21 = null;
        try {
            r21toCoralL = PathPlannerPath.fromPathFile("R21 to CoralL");
            coralLToR21 = PathPlannerPath.fromPathFile("CoralL to R21");
        }
        catch(Exception e){}

        return Commands.sequence(
            autoAlignCommand,
            scoreCommand,
            AutoBuilder.followPath(r21toCoralL),
            AutoBuilder.followPath(coralLToR21), 
            autoAlignCommand,
            scoreCommand
        );
    }
}
