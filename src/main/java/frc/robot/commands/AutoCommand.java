package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
public class AutoCommand extends SequentialCommandGroup{
    
    
    PathPlannerPath path;
    public AutoCommand(AutoAlignCommand autoAlignCommand){
        try{
             path = PathPlannerPath.fromPathFile("Example Path");
        }
        catch(Exception e){
            path = null;
        }


        addCommands(autoAlignCommand,AutoBuilder.followPath(path));

    }
    
}
