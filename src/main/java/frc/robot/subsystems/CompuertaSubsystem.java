
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompuertaSubsystem extends SubsystemBase {

Solenoid compuertaSolenoid=new Solenoid(PneumaticsModuleType.CTREPCM, 1);

  public CompuertaSubsystem() {


  }

  @Override
  public void periodic() {

  }

public void compuertastatus(boolean state){

compuertaSolenoid.set(state);

}


}
