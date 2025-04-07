package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TeleopAutoAlignCommand extends Command{
    Supplier<Optional<Pose2d>> poseSupplier;
    int targetId;
    boolean left;
    SwerveSubsystem swerve;
    VisionSubsystem vision;

    public Pose2d distFromTag;
    private PIDController xController = new PIDController(Constants.SwerveConstants.kPX, 0, 0);
    private PIDController yController = new PIDController(Constants.SwerveConstants.kPY, 0, 0);
    private PIDController thetaController = new PIDController(Constants.SwerveConstants.kPTheta, 0, 0);


    public TeleopAutoAlignCommand(boolean left, SwerveSubsystem swerve, VisionSubsystem vision){
        addRequirements(swerve);
        this.left = left;
        this.swerve = swerve;
        this.vision = vision;

    }

    @Override
    public void initialize() {
        this.targetId = vision.getBestTag(left);
        System.out.println(targetId);

    }

    @Override
    public void execute(){
        if(targetId ==-1){targetId=vision.getBestTag(left);return;}
        distFromTag = vision.getRobotToTagTransform(left, targetId);
        if(distFromTag==null)return;
      double xPower = MathUtil.clamp(xController.calculate(distFromTag.getX(), left ? Constants.VisionConstants.leftAlignmentX : Constants.VisionConstants.rightAlignmentX), -1.5, 1.5);
      double yPower = MathUtil.clamp(yController.calculate(distFromTag.getY(), left ? Constants.VisionConstants.leftAlignmentY : Constants.VisionConstants.rightAlignmentY), -1.5, 1.5);
      double thetaPower = thetaController.calculate(distFromTag.getRotation().getRadians(), Constants.VisionConstants.thetaAlignment);
      SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentX",distFromTag.getX());
      SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentY",distFromTag.getY());
      SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentTheta",distFromTag.getRotation().getRadians());
      SmartDashboard.putNumber("Subsystem/Vision/xPOut",xPower);
      SmartDashboard.putNumber("Subsystem/Vision/yPOut",yPower);
      SmartDashboard.putNumber("Subsystem/Vision/thetaOut",thetaPower);
       swerve.drive(new ChassisSpeeds(-yPower, xPower, thetaPower));
    }
    




    @Override
    public boolean isFinished(){
        return (Math.abs(distFromTag.getX()-(left ? Constants.VisionConstants.leftAlignmentX : Constants.VisionConstants.rightAlignmentX))<VisionConstants.xTolerance) 
        && (Math.abs(distFromTag.getY()-(left ? Constants.VisionConstants.leftAlignmentY : Constants.VisionConstants.rightAlignmentY))<VisionConstants.yTolerance)
        && (Math.abs(distFromTag.getRotation().getRadians()-Constants.VisionConstants.thetaAlignment)<VisionConstants.thetaTolerance); 
    } 

}
