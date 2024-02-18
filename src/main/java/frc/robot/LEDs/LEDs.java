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

    m_LED.setData(m_LEDBuffer);
    m_LED.start();
  }

  private void setColor(Color color) {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setLED(i, color);
    }
    m_LED.setData(m_LEDBuffer);
  }

  private void turnOffLEDs() {
    clearBuffer();
    m_LED.setData(m_LEDBuffer);
  }

  private void clearBuffer() {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setRGB(i, 0, 0, 0);
    }
  }

  private void autoColor(boolean isRed, int autoMode) {
    clearBuffer();
    int block = LEDConstants.kLEDsPerBlock + LEDConstants.kLEDsBetweenBlocks;
    for (var mode = 0; mode < autoMode; mode++) {
      for (var i = 0; i < LEDConstants.kLEDsPerBlock; i++) {
        if (isRed) {
          m_LEDBuffer.setLED(i + (mode * block), Color.kRed);
        } else {
          m_LEDBuffer.setLED(i + (mode * block), Color.kBlue);
        }
      }
    }
    m_LED.setData(m_LEDBuffer);
  }

  private void displayGamePieceDetected(boolean hasGamePiece) {
    if (hasGamePiece) setColor(Color.kGreen);
    else turnOffLEDs();
  }

  public Command createTeleopCommand(BooleanSupplier gamePieceSensor) {
    return this.run(() -> this.displayGamePieceDetected(gamePieceSensor.getAsBoolean()))
        .ignoringDisable(true);
  }

  public Command createDisabledCommand(
      BooleanSupplier redAllianceSupplier, IntSupplier autoModeSwitch) {
    return this.run(
            () -> this.autoColor(redAllianceSupplier.getAsBoolean(), autoModeSwitch.getAsInt()))
        .ignoringDisable(true);
  }
}
