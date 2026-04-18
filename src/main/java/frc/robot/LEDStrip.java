package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDStrip {
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    public static final LEDPattern patternWhite = LEDPattern.solid(Color.kWhite);
    public static final LEDPattern patternYellow = LEDPattern.solid(Color.kYellow);
    public static final LEDPattern patternBlue = LEDPattern.solid(Color.kBlue);
    public static final LEDPattern patternOff = LEDPattern.solid(Color.kBlack);

    public LEDStrip(int port, int length) {
        this.led = new AddressableLED(port);
        this.ledBuffer = new AddressableLEDBuffer(length);
        this.led.setLength(length);
        this.led.setData(this.ledBuffer);
        this.led.start();
    }

    public void setLEDs(char patternId) {
        LEDPattern pattern = patternOff;

        if (patternId == 'Y') {
            pattern = patternYellow;
        } else if (patternId == 'B') {
            pattern = patternBlue;
        } else if (patternId == 'W') {
            pattern = patternWhite;
        }

        pattern.applyTo(this.ledBuffer);
        this.led.setData(this.ledBuffer);
    }
}
