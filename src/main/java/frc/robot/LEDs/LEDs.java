package frc.robot.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants;

public class LEDs {
  private final AddressableLED m_LED = new AddressableLED(LEDConstants.kLEDPort);
  private final AddressableLEDBuffer m_LEDBuffer =
      new AddressableLEDBuffer(LEDConstants.kLEDLength);

  public LEDs() {
    m_LED.setLength(m_LEDBuffer.getLength());
  }

  public void setColorYellow() {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setLED(i, Color.kYellow);
    }
  }

  public void setColorPurple() {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setLED(i, Color.kPurple);
    }
  }

  public void setColorGreen() {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setLED(i, Color.kGreen);
    }
  }

  public void turnOffLEDs() {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setRGB(i, 0, 0, 0);
    }
  }

  public void autoColor(boolean isBlue, int autoMode) {
    for (var mode = 0; mode < autoMode; mode++) {
      for (var i = 0; i < LEDConstants.kChunkSize + LEDConstants.kLEDSpacing; i++) {
        if (isBlue) {
          m_LEDBuffer.setLED(i, Color.kBlue);
        } else {
          m_LEDBuffer.setLED(i, Color.kRed);
        }
      }
    }
  }
}
