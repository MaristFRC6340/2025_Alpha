package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;


public class ElevatorSubsystem extends SubsystemBase{

    TalonFX leftMotor;
    TalonFX rightMotor;

    MutVoltage appliedVoltage = Volts.mutable(0);
    MutDistance distance = Meters.mutable(0);
    MutLinearVelocity vel = MetersPerSecond.mutable(0);
    

    private final MotionMagicExpoVoltage m_MMEV= new MotionMagicExpoVoltage(0); //Position in rotations
    private final Follower m_Follower;
    private final VoltageOut voltageOutput;
    private final DutyCycleOut speedRequest = new DutyCycleOut(0);

    

    public ElevatorSubsystem() {

        leftMotor = new TalonFX(Constants.ElevatorConstants.kLeftID);
        leftMotor.getConfigurator().apply(Constants.ElevatorConstants.kLeftConfig);

        rightMotor = new TalonFX(Constants.ElevatorConstants.kRightID);
        rightMotor.getConfigurator().apply(Constants.ElevatorConstants.kRightConfig);
        //instantiate control modes

        voltageOutput= new VoltageOut(appliedVoltage);
        m_Follower = new Follower(leftMotor.getDeviceID(), true); // follows movements of right motor

        leftMotor.setControl(m_MMEV);
        rightMotor.setControl(m_Follower);
        //SignalLogger.start();
        
    }

    // public Command requestElevation() {
    //     return this.runOnce(() -> elevate());
    // }

    // public void elevate() { // idk what i'm doing :sob:
    // // we're basically setting velocity using motion magic
    //     final MotionMagicVoltage m_vrequest = new MotionMagicVoltage(0); // create a req
    //     // insert code here that lets us control the pos via Triggers(?)
    //     leftMotor.setControl(m_vrequest.withPosition(80)); // not sure what is a good value for pos
    //     rightMotor.setControl(m_Follower); // follows left motor
    // }
    
    public SysIdRoutine getRoutine(){
      return 
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                null,          // Use default timeout (10 s)
                                       // Log state with Phoenix SignalLogger class
                state -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> leftMotor.setControl(voltageOutput.withOutput(volts)),
                null,
                this
            )
        );
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Subsystem/Elevator/LeftMotor/position", leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Subsystem/Elevator/LeftMotor/angle", leftMotor.getPosition().getValue().magnitude());
        SmartDashboard.putNumber("Subsystem/Elevator/LeftMotor/voltage2", leftMotor.getSupplyVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Subsystem/Elevator/LeftMotor/voltage", leftMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Subsystem/Elevator/LeftMotor/velocity", leftMotor.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("Subsystem/Elevator/RightMotor/position", rightMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Subsystem/Elevator/RightMotor/angle", rightMotor.getPosition().getValue().magnitude());
        SmartDashboard.putNumber("Subsystem/Elevator/RightMotor/voltage", rightMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Subsystem/Elevator/RightMotor/velocity", rightMotor.getVelocity().getValueAsDouble());
        
        //SmartDashboard.putData("Subsystem/Elevator/currentCommand", this.getCurrentCommand());
        //SmartDashboard.putData("Subsystem/Elevator/defaultCommand", this.getDefaultCommand());
    }

    public void setVoltage(Voltage v){
        leftMotor.setControl(voltageOutput.withOutput(v));
    }

    public void resetEncoder(){
        this.getCurrentCommand().cancel();
        leftMotor.setControl(speedRequest.withOutput(0));
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
        setPosition(0);
    }
    //Clamps the position value
    public void setPosition(double position) {
        position = MathUtil.clamp(position,Constants.ElevatorConstants.kMin,Constants.ElevatorConstants.kMax);
        leftMotor.setControl(m_MMEV.withPosition(position));
        rightMotor.setControl(m_Follower); // could be needed (or not)
    }
    

    public Command getSetPositionCommand(double position) {
        return this.runOnce(() -> {
            setPosition(position);
        });
    }

    public Command setPower(DoubleSupplier dubSupp){
        return this.runEnd(()->{
            if(!((getPosition()>=ElevatorConstants.kMax&&dubSupp.getAsDouble()>0) ||(getPosition()<=ElevatorConstants.kMin && dubSupp.getAsDouble()<0))){
                leftMotor.setControl(speedRequest.withOutput(dubSupp.getAsDouble()));
            }
            else{
                setPosition(getPosition());
            }
        },()->{
            setPosition(getPosition());
        });
    }
    public double getPosition(){
        return leftMotor.getPosition().getValueAsDouble();
    }

    public Command goHome() {
        return getSetPositionCommand(0);
    }
}
