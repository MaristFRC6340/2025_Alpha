package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HuggerSubsystem extends SubsystemBase{

   
    private TalonFX pivotMotor;

    private TalonFX leftMotor;
    //private TalonFX rightMotor;

    private PositionVoltage m_PositionVoltage = new PositionVoltage(0); //Position in rotations
    private PositionVoltage huggerHold = new PositionVoltage(0); //Position in rotations

    public HuggerSubsystem() {

        //Pivot Motor
        pivotMotor = new TalonFX(Constants.HuggerConstants.kPivotID);
        Slot0Configs kPivotConfig = new Slot0Configs();
        kPivotConfig.kP=Constants.HuggerConstants.kPPivot;
        kPivotConfig.kG=Constants.HuggerConstants.kGPivot;
        pivotMotor.setNeutralMode(NeutralModeValue.Brake); // From Mr Michaud: Should this be "brake"?
        FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs().withRotorToSensorRatio(36);
        pivotMotor.getConfigurator().apply(kPivotConfig);
        pivotMotor.getConfigurator().apply(kFeedbackConfigs);

        //left and right motors

        leftMotor = new TalonFX(Constants.HuggerConstants.kLeftID);
        leftMotor.getConfigurator().apply(Constants.HuggerConstants.leftMotorConfig);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);


    }

    @Override 
    public void periodic(){
        SmartDashboard.putNumber("/Subsystem/Hugger/Pivot/position",pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("/Subsystem/Hugger/Pivot/velocity", pivotMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("/Subsystem/Hugger/Pivot/angle", pivotMotor.getPosition().getValue().magnitude());
        SmartDashboard.putNumber("/Subsystem/Hugger/Pivot/positionVoltageTarget", m_PositionVoltage.Position);

        SmartDashboard.putNumber("/Subsystem/Hugger/LeftMotor/position",leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("/Subsystem/Hugger/LeftMotor/velocity",leftMotor.getPosition().getValueAsDouble());
        
        
        // SmartDashboard.putData("Subsystem/Coral/currentCommand", this.getCurrentCommand());
        // SmartDashboard.putData("Subsystem/Coral/defaultCommand", this.getDefaultCommand());      

    }

    public void holdHuggerWheelPosition(){
        leftMotor.setControl(huggerHold.withPosition(leftMotor.getPosition().getValueAsDouble()));
    }
    public void setPivotPower(double power){
        pivotMotor.set(power);
    }

    public Command getSetPivotPower(DoubleSupplier supp){
        System.out.println(supp.getAsDouble());

        return this.runEnd(()->{
            setPivotPower(supp.getAsDouble());


        }, ()->{
            setPosition(getPosition());
            //setPosition(getPosition());
        });
    }

    /**
     * 
     * @param position the position in rotations
     * @return command to set position
     */
    public void setPosition(double position) {
        //position = MathUtil.clamp(position, 0, 0);
        
            pivotMotor.setControl(m_PositionVoltage.withPosition(position));
            System.out.println(position);
    }

    public Command getSetSpeedCommand(DoubleSupplier speed) {
        return this.startEnd(() -> {
            setSpeed(speed.getAsDouble());
        }, () -> {
            holdHuggerWheelPosition();
        });
    }

    public void setSpeed(double speed) {
        leftMotor.set(speed);
        //rightMotor.set(-speed);
    }
    public void resetPivotEncoder(){
        pivotMotor.setPosition(0);
    }
    
    public void stop() {
        setSpeed(0);
    }
    public double getPosition(){
        return pivotMotor.getPosition().getValueAsDouble();
    }
}
