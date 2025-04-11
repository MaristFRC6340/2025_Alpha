package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.led.FireAnimation;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{

   
    AddressableLED strip;
    AddressableLEDBuffer buff;
    
    public LEDSubsystem(){
        strip = new AddressableLED(9);
        buff = new AddressableLEDBuffer(100);
        strip.setLength(buff.getLength());

        
        strip.setData(buff);
        strip.start();
        setDefaultCommand(setPattern(LEDPattern.solid(Color.kBlue)));
        //setDefaultCommand(setPattern(this::setFlamePattern));

    }
    @Override
    public void periodic(){
        strip.setData(buff);
        if(this.getCurrentCommand() != null){
            SmartDashboard.putData("LEDCOmmand",this.getCurrentCommand());
        }
    }



    public Command setPattern(LEDPattern pattern){
        return this.run(()->pattern.applyTo(buff));
    }
    public Command setPattern(Runnable setBuffer){
        return this.run(()->setBuffer.run());
    }

    

    
}
