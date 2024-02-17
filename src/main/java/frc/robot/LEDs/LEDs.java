package frc.robot.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private void setColor(Color color) {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setLED(i, color);
    }
  }

  private void turnOffLEDs() {
    for (var i = 0; i < m_LEDBuffer.getLength(); i++) {
      m_LEDBuffer.setRGB(i, 0, 0, 0);
    }
  }

  private void autoColor(boolean isRed, int autoMode) {
    SmartDashboard.putString("LED Mode", "Displaying autonomous mode choice");
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

  private void displayGamePieceDetected(boolean hasGamePiece) {
    SmartDashboard.putString("LED Mode", "Displaying game piece detection state");
    if (hasGamePiece) setColor(Color.kGreen);
    else turnOffLEDs();
  }

  public Command createTeleopCommand(BooleanSupplier gamePieceSensor) {
    return this.run(() -> this.displayGamePieceDetected(gamePieceSensor.getAsBoolean())).ignoringDisable(true);
  }

  public Command createDisabledCommand(
      BooleanSupplier redAllianceSupplier, IntSupplier autoModeSwitch) {
    return this.run(
        () -> this.autoColor(redAllianceSupplier.getAsBoolean(), autoModeSwitch.getAsInt())).ignoringDisable(true);
  }
}
