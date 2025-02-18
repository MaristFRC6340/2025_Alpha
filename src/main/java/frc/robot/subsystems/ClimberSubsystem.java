package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{
    
    //Fields
    TalonFX climberMotor;
    Slot0Configs slot0config;
    PositionVoltage p = new PositionVoltage(0).withSlot(0);

    //Constructor
    public ClimberSubsystem() {

        climberMotor = new TalonFX(Constants.ClimberConstants.kClimberId);
        climberMotor.getConfigurator().apply(Constants.ClimberConstants.kClimberConfig);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);


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

    public Command setPower(double power) {
        return this.startEnd(() -> {
            climberMotor.set(power);
        }, () -> {
            climberMotor.set(0);
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
        // SmartDashboard.putData("Subsystem/Climber/currentCommand", this.getCurrentCommand());
        // SmartDashboard.putData("Subsystem/Climber/defaultCommand", this.getDefaultCommand());
    }
}
