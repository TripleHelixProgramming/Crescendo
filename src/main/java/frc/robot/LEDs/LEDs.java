package frc.robot.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

public class LEDs extends SubsystemBase {
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

  public void autoColor(boolean isRed, int autoMode) {
    int LEDChunk = LEDConstants.kLightSpaces + LEDConstants.kLEDSpacing;
    for (var mode = 0; mode < autoMode; mode++) {
      for (var i = 0; i < LEDConstants.kLightSpaces; i++) {
        if (isRed) {
          m_LEDBuffer.setLED(i + (mode * LEDChunk), Color.kRed);
        } else {
          m_LEDBuffer.setLED(i + (mode * LEDChunk), Color.kBlue);
        }
      }
    }
  }

  public void displayGamePieceDetected(boolean hasGamePiece) {
    if (hasGamePiece) setColorGreen();
    else turnOffLEDs();
  }

  public Command createTeleopCommand(BooleanSupplier gamePieceSensor) {
    return this.run(() -> displayGamePieceDetected(gamePieceSensor.getAsBoolean()));
  }

  public Command createAutonomousCommand(
      BooleanSupplier redAllianceSupplier, IntSupplier autoModeSwitch) {
    return this.run(
        () -> this.autoColor(redAllianceSupplier.getAsBoolean(), autoModeSwitch.getAsInt()));
  }
}
