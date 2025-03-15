package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignCommand extends Command{
    Supplier<Optional<Pose2d>> poseSupplier;
    IntSupplier idSupplier;
    boolean left;
    SwerveSubsystem swerve;
    public Pose2d distFromTag;
    private PIDController xController = new PIDController(Constants.SwerveConstants.kPX, 0, 0);
    private PIDController yController = new PIDController(Constants.SwerveConstants.kPY, 0, 0);
    private PIDController thetaController = new PIDController(Constants.SwerveConstants.kPTheta, 0, 0);


    public AutoAlignCommand(Supplier<Optional<Pose2d>> poseSupplier,  IntSupplier idSupplier, boolean left, SwerveSubsystem swerve){
        this.poseSupplier = poseSupplier;
        this.idSupplier = idSupplier;
        this.left = left;
        this.swerve = swerve;
    }



    @Override
    public void execute(){
        Optional<Pose2d> currentPoseOpt = poseSupplier.get();
    if(currentPoseOpt.isPresent()) {
      Pose2d currentPose = currentPoseOpt.get();
      distFromTag = currentPose;

      double xPower = xController.calculate(currentPose.getX(), left ? Constants.VisionConstants.leftAlignmentX : Constants.VisionConstants.rightAlignmentX);
      double yPower = yController.calculate(currentPose.getY(), left ? Constants.VisionConstants.leftAlignmentY : Constants.VisionConstants.rightAlignmentY);
      double thetaPower = thetaController.calculate(currentPose.getRotation().getRadians(), Constants.VisionConstants.thetaAlignment);
    //   SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentX",currentPose.getX());
    //   SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentY",currentPose.getY());
    //   SmartDashboard.putNumber("Subsystem/Vision/actualAlignmentTheta",currentPose.getRotation().getRadians());
    //   SmartDashboard.putNumber("Subsystem/Vision/xPOut",xPower);
    //   SmartDashboard.putNumber("Subsystem/Vision/yPOut",yPower);
      //SmartDashboard.putNumber("Subsystem/Vision/thetaOut",thetaPower);
      swerve.drive(new ChassisSpeeds(-yPower, xPower, thetaPower));
     // System.out.println("in");
    }
    //System.out.println("out");

}

    @Override
    public boolean isFinished(){
        return (Math.abs(distFromTag.getX()-(left ? Constants.VisionConstants.leftAlignmentX : Constants.VisionConstants.rightAlignmentX))<.05) 
        && (Math.abs(distFromTag.getY()-(left ? Constants.VisionConstants.leftAlignmentY : Constants.VisionConstants.rightAlignmentY))<.05)
        && (Math.abs(distFromTag.getRotation().getRadians()-Constants.VisionConstants.thetaAlignment)<.02)

        ;

    } 
}
