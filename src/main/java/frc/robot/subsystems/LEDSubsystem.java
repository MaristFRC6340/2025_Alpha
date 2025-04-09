package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.led.FireAnimation;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{

   
    AddressableLED strip;
    AddressableLEDBuffer buff;
    
    public LEDSubsystem(){
        strip = new AddressableLED(9);
        buff = new AddressableLEDBuffer(26);
        strip.setLength(buff.getLength());

        
        strip.setData(buff);
        strip.start();
        //setDefaultCommand(setPattern(LEDPattern.solid(Color.kBlue)));
        setDefaultCommand(setPattern(this::setFlamePattern));

    }
    @Override
    public void periodic(){
        strip.setData(buff);
    }



    public Command setPattern(LEDPattern pattern){
        return this.run(()->pattern.applyTo(buff));
    }
    public Command setPattern(Runnable setBuffer){
        return this.run(()->setBuffer.run());
    }

    private void setFlamePattern() {
        for (int i = 0; i < buff.getLength(); i++) {
            // Simulate random flickering (variation in brightness and colors)
            int flicker = (int)(Math.random() * 50); // Random flicker intensity

            // Flame colors: Red, Orange, Yellow (adjust as needed)
            int baseRed = 255; // Red component (constant for flame)
            int baseGreen = (int)(Math.random() * 50) + 50; // Green component (for yellow or orange hues)
            int baseBlue = (int)(Math.random() * 50); // Low blue component for flame effect

            // Apply flicker by adjusting brightness
            baseRed = Math.max(0, baseRed - flicker);
            baseGreen = Math.max(0, baseGreen - flicker / 2);

            // Set the color of each LED (you can adjust this to add more complex effects)
            buff.setLED(i, new Color(baseRed, baseGreen, baseBlue));
        }
    }

    
}
