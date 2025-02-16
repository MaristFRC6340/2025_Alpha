package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
@Logged

public class ClimberSubsystem extends SubsystemBase{
    
    //Fields
    TalonFX climberMotor;
    Slot0Configs slot0config;
    Servo ratchetServo;
    PositionVoltage p = new PositionVoltage(0).withSlot(0);

    //Constructor
    public ClimberSubsystem() {

        climberMotor = new TalonFX(Constants.ClimberConstants.kClimberId);
        climberMotor.getConfigurator().apply(Constants.ClimberConstants.kClimberConfig);

        ratchetServo = new Servo(ClimberConstants.kServoID);

    }

    /**
     * 
     */
    public void ratchetOn() {
        ratchetServo.set(Constants.ClimberConstants.kRatchetOn);
    }

    public void ratchetOff() {
        ratchetServo.set(Constants.ClimberConstants.kRatchetOff);
    }
    /**
     * 
     * @param pow - the power supply on the robot
     * @return
     */
    public Command setPower(DoubleSupplier pow){
        return this.runEnd(()->{
            climberMotor.set(pow.getAsDouble());
        },()->{
           climberMotor.setControl(p.withPosition(climberMotor.getPosition().getValueAsDouble()));
        });
    }

    /**
     * 
     * @param position - position on the field
     * @return
     */
    public Command setPosition(double position){
        return this.runOnce(()->{
            climberMotor.setControl(p.withPosition(position));
        });
    }
    public void resetEncoder(){
        //this.getCurrentCommand().cancel();
        climberMotor.setPosition(0);
    }

    public void periodic() {
        SmartDashboard.putNumber("Subsystem/Climber/Motor/position", climberMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Subsystem/Climber/Motor/angle", climberMotor.getPosition().getValue().magnitude());
        SmartDashboard.putNumber("Subsystem/Climber/Motor/voltage", climberMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Subsystem/Climber/Servo/position", ratchetServo.getPosition());
        SmartDashboard.putNumber("Subsystem/Climber/Servo/angle", ratchetServo.getAngle());
        SmartDashboard.putNumber("Subsystem/Climber/Servo/speed", ratchetServo.getSpeed());
        SmartDashboard.putData("Subsystem/Climber/currentCommand", this.getCurrentCommand());
        SmartDashboard.putData("Subsystem/Climber/defaultCommand", this.getDefaultCommand());
    }
}
