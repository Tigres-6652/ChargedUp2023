
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KPIDejeinferior;
import frc.robot.Constants.KPIDejesuperior;
import frc.robot.Constants.PUERTOSCAN;
import frc.robot.Constants.posicionesbrazo.topesvelocidad;

public class BrazoSubsystem extends SubsystemBase {

  WPI_TalonSRX motejeabajo = new WPI_TalonSRX(PUERTOSCAN.PuertMotEjeinferior);
  WPI_TalonFX motejearriba = new WPI_TalonFX(PUERTOSCAN.PuertMotEjeSuperior);

  DigitalInput limitejeinferior = new DigitalInput(8);
  DigitalInput limitejesuperior = new DigitalInput(9);

  public BrazoSubsystem() {
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("limitarriba", !limitejesuperior.get());
    SmartDashboard.putBoolean("limitinferior", !limitejeinferior.get());
    SmartDashboard.putNumber("grados eje inferior", gradosEjeInferior());
    SmartDashboard.putNumber("grados eje superior", gradosEjeSuperior());

  }

  public void ejeinferior(double velinf) {  //movimiento con control

    if (limitejeinferior.get()&&gradosEjeInferior()<70) {

      motejeabajo.set(velinf);

    } else if (!limitejeinferior.get() && velinf < 0.001) {

      motejeabajo.set(velinf);

    } else if (!limitejeinferior.get() && velinf > 0.001) {

      motejeabajo.set(0);

    }else if(gradosEjeInferior()>70&&velinf < 0.001){
      motejeabajo.set(0);

    } else if(gradosEjeInferior()>70&&velinf > 0.001&&limitejeinferior.get()){
      motejeabajo.set(velinf);

    }
    if (!limitejeinferior.get()) {

      motejeabajo.setSelectedSensorPosition(0);

    } 
  }

  public void ejesuperior(double velsup) { //movimiento con control

    if (limitejesuperior.get()&&gradosEjeSuperior()>-250) {

      motejearriba.set(velsup);

    } else if (!limitejesuperior.get() && velsup < 0.001) {

      motejearriba.set(velsup);

    } else if (!limitejesuperior.get() && velsup > 0.001) {

      motejearriba.set(0);
    } else if(gradosEjeSuperior()<-250&&velsup>0.001&&limitejesuperior.get()){
      motejearriba.set(velsup);
    } else if(gradosEjeSuperior()<-250&&velsup<0.001&&limitejesuperior.get()){
      motejearriba.set(0);

    }

    if (!limitejesuperior.get()) {

      motejearriba.setSelectedSensorPosition(0);

    }

    // motejearriba.set(TalonFXControlMode.Velocity, (velocidadpadarle) )

  }

  public double gradosEjeSuperior() {

    double pulsosSensor = motejearriba.getSelectedSensorPosition();

    double vuelta_eje_con_reduccion = ((pulsosSensor) / (4096) / (100));

    double gradoseje = (vuelta_eje_con_reduccion / 3.285714) * (360) * 2;

    return gradoseje;
  }

  public double gradosEjeInferior() {

    double pulsosSensor = motejeabajo.getSelectedSensorPosition();

    double vuelta_eje_con_reduccion = ((pulsosSensor) / (4096));

    double gradoseje = (vuelta_eje_con_reduccion / 4.5714) * (360);

    return gradoseje;

  }

  public void returnhome(boolean status) {

    if (status) {

      if (!limitejeinferior.get()) {

        motejeabajo.set(0);

      } else {

        motejeabajo.set(0.4);

      }

      if (!limitejesuperior.get()) {

        motejearriba.set(0);

      } else {
        motejearriba.set(0.4);

      }

    }

  }

public void movimiento_brazo_angulo(double gradosinferior, double gradosuperior){

double movimientosuperior=(gradosEjeSuperior()-gradosuperior)*0.04;
double movimientoinferior=(gradosEjeInferior()-gradosinferior)*0.03;


double conversiontopesuperior;
double conversiontopeinferior;


if(movimientoinferior<-topesvelocidad.Inferior){
  conversiontopeinferior=-topesvelocidad.Inferior;
}else if(movimientoinferior>topesvelocidad.Inferior){
  conversiontopeinferior=topesvelocidad.Inferior;
}else{
  conversiontopeinferior=movimientoinferior;
}


if(movimientosuperior>topesvelocidad.Superior){
  conversiontopesuperior=topesvelocidad.Superior;
}else if(movimientosuperior<-topesvelocidad.Superior){
  conversiontopesuperior=-topesvelocidad.Superior;
}else{
  conversiontopesuperior=movimientosuperior;

}


motejearriba.set(-conversiontopesuperior);
motejeabajo.set(conversiontopeinferior);

}


public void ResetEncoderLimit(){

  if (!limitejesuperior.get()) {

    motejearriba.setSelectedSensorPosition(0);

  }

  if (!limitejeinferior.get()) {

    motejeabajo.setSelectedSensorPosition(0);

  }
}





  //Configuracion de motores para que no se muevan mucho al momento de tener el robot encendido
// *NO MOVERLE*
  public void config_motor_eje_inf() {

      motejeabajo.configNominalOutputForward(0, KPIDejeinferior.kTimeoutMs);
      motejeabajo.configNominalOutputReverse(0, KPIDejeinferior.kTimeoutMs);
      motejeabajo.configPeakOutputForward(1, KPIDejeinferior.kTimeoutMs);
      motejeabajo.configPeakOutputReverse(-1, KPIDejeinferior.kTimeoutMs);
      /* Config the Velocity closed loop gains in slot0 */
      motejeabajo.config_kF(KPIDejeinferior.kPIDLoopIdx, KPIDejeinferior.kF,
          KPIDejeinferior.kTimeoutMs);
      motejeabajo.config_kP(KPIDejeinferior.kPIDLoopIdx, KPIDejeinferior.KP,
          KPIDejeinferior.kTimeoutMs);
      motejeabajo.config_kI(KPIDejeinferior.kPIDLoopIdx, KPIDejeinferior.kI,
          KPIDejeinferior.kTimeoutMs);
      motejeabajo.config_kD(KPIDejeinferior.kPIDLoopIdx, KPIDejeinferior.kD,
          KPIDejeinferior.kTimeoutMs);

  }

  //Configuracion de motores para que no se muevan mucho al momento de tener el robot encendido
  // *NO MOVERLE*
  public void config_motor_eje_sup() {

    motejearriba.configNominalOutputForward(0, KPIDejesuperior.kTimeoutMs);
    motejearriba.configNominalOutputReverse(0, KPIDejesuperior.kTimeoutMs);
    motejearriba.configPeakOutputForward(1, KPIDejesuperior.kTimeoutMs);
    motejearriba.configPeakOutputReverse(-1, KPIDejesuperior.kTimeoutMs);
    /* Config the Velocity closed loop gains in slot0 */
    motejearriba.config_kF(KPIDejesuperior.kPIDLoopIdx, KPIDejesuperior.kF,
        KPIDejesuperior.kTimeoutMs);
    motejearriba.config_kP(KPIDejesuperior.kPIDLoopIdx, KPIDejesuperior.KP,
        KPIDejesuperior.kTimeoutMs);
    motejearriba.config_kI(KPIDejesuperior.kPIDLoopIdx, KPIDejesuperior.kI,
        KPIDejesuperior.kTimeoutMs);
    motejearriba.config_kD(KPIDejesuperior.kPIDLoopIdx, KPIDejesuperior.kD,
        KPIDejesuperior.kTimeoutMs);
  }

  /*
   * Engranes Robot, conversion encorders:
   * 
   * Eje inferior:
   * 
   * 14 dientes en motor
   * 64 dientes
   * 
   * 
   * 
   * 4.5714
   * 
   * 
   * 
   * 
   * 
   * Eje superior:
   * 
   * 46 dientes
   * 14 dientes a motor
   * 
   * reduccion 100
   * 
   * 3.285714
   * 
   */

}
