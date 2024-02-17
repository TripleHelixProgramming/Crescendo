package frc.robot.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LEDConstants;;

public class LEDs {
    private final AddressableLED m_LED = new AddressableLED(LEDConstants.kLEDPort);
    private final AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

    public LEDs() {
        m_LED.setLength(m_LEDBuffer.getLength());
        
    }
}