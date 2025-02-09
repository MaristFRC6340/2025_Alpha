package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

@Logged

public class ElevatorSubsystem extends SubsystemBase{

    TalonFX leftMotor;
    TalonFX rightMotor;

    MutVoltage appliedVoltage = Volts.mutable(0);
    MutDistance distance = Meters.mutable(0);
    MutLinearVelocity vel = MetersPerSecond.mutable(0);
    

    private final MotionMagicExpoVoltage m_MMEV= new MotionMagicExpoVoltage(0); //Position in rotations
    private final Follower m_Follower;
    private final VoltageOut voltageOutput;

    

    public ElevatorSubsystem() {

        leftMotor = new TalonFX(Constants.ElevatorConstants.kLeftID);
        leftMotor.getConfigurator().apply(Constants.ElevatorConstants.kLeftConfig);

        rightMotor = new TalonFX(Constants.ElevatorConstants.kRightID);
        rightMotor.getConfigurator().apply(Constants.ElevatorConstants.kRightConfig);
        //instantiate control modes

        voltageOutput= new VoltageOut(appliedVoltage);
        m_Follower = new Follower(leftMotor.getDeviceID(), true);

        leftMotor.setControl(m_MMEV);
        rightMotor.setControl(m_Follower);
        
    }
    

    // public SysIdRoutine getSysIdRouine(){
    //     return new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism((voltage)->{setVoltage(voltage);}, (log)->
    //     {
    //     log.motor("Elevator_Right")
    //     .voltage(appliedVoltage.mut_replace(rightMotor.get()*RobotController.getBatteryVoltage(), Volts))
    //     .linearPosition(distance.mut_replace(rightMotor.getPosition().getValueAsDouble(),Meters))
    //     .linearVelocity(vel.mut_replace(rightMotor.getVelocity().getValueAsDouble(),MetersPerSecond));


    //     log.motor("Elevator_Left")
    //     .voltage(appliedVoltage.mut_replace(leftMotor.get()*RobotController.getBatteryVoltage(), Volts))
    //     .linearPosition(distance.mut_replace(leftMotor.getPosition().getValueAsDouble(),Meters))
    //     .linearVelocity(vel.mut_replace(leftMotor.getVelocity().getValueAsDouble(),MetersPerSecond));
    // }, this));
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

    public void setVoltage(Voltage v){
        leftMotor.setControl(voltageOutput.withOutput(v));
    }

    //Clamps the position value
    public void setPosition(double position) {
        leftMotor.setControl(m_MMEV.withPosition(Math.max(Constants.ElevatorConstants.kMin, Math.min(Constants.ElevatorConstants.kMax, position)) ));
    }
    

    public Command getSetPositionCommand(double position) {
        return this.runOnce(() -> {
            setPosition(position);
        });
    }

    public Command goHome() {
        return getSetPositionCommand(0);
    }
}
