
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompresorSubsystem extends SubsystemBase {

  Compressor compr = new Compressor(0, PneumaticsModuleType.CTREPCM);

  public CompresorSubsystem() {
  }

  @Override
  public void periodic() {
  }

  public void Compresor(boolean state) {
    SmartDashboard.putBoolean("Compressor", state);
    if (state == false) {
      compr.disable();

    } else {
      compr.enableDigital();

    }

  }
}
