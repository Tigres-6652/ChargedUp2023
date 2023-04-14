
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.posicionesbrazo.posicionSafeArm;
import frc.robot.Constants.posicionesbrazo.posicioncubopiso;
import frc.robot.Constants.posicionesbrazo.posiciondejarAbajo;
import frc.robot.Constants.posicionesbrazo.posiciondejarCubo;
import frc.robot.Constants.posicionesbrazo.posiciondejarCuboArriba;
import frc.robot.Constants.posicionesbrazo.posiciondejarcono;
import frc.robot.Constants.posicionesbrazo.posicionConodoublesubstation;
import frc.robot.Constants.posicionesbrazo.posicionCuboDoubleSub;
import frc.robot.subsystems.BrazoSubsystem;

public class BrazoCmd extends CommandBase {

  private final BrazoSubsystem brazosubsystem;
  private Supplier<Double> velocidadInferior, velocidadSuperior;
  private Supplier<Boolean> ReturnHome, ModoSubstationCono, SafeArm, DejarConoEnmedio,
      BotonDesbloqueo, ModoRecogePiso, ModoSubstationCubo, DejarCubomid, DejarCuboarriba, Dejarabajo;

  public BrazoCmd(BrazoSubsystem brazosubsystem, Supplier<Double> velocidadInferior, Supplier<Double> velocidadSuperior,
      Supplier<Boolean> BotonDesbloqueo, Supplier<Boolean> ReturnHome, Supplier<Boolean> ModoSubstationCono,
      Supplier<Boolean> SafeArm, Supplier<Boolean> DejarConoEnmedio, Supplier<Boolean> ModoRecogePiso,
      Supplier<Boolean> ModoSubstationCubo, Supplier<Boolean> DejarCubomid, Supplier<Boolean> DejarCuboarriba,
      Supplier<Boolean> Dejarabajok) {

    this.ReturnHome = ReturnHome;
    this.ModoSubstationCono = ModoSubstationCono;
    this.SafeArm = SafeArm;
    this.DejarConoEnmedio = DejarConoEnmedio;
    this.BotonDesbloqueo = BotonDesbloqueo;
    this.ModoRecogePiso = ModoRecogePiso;
    this.DejarCubomid = DejarCubomid;
    this.DejarCuboarriba = DejarCuboarriba;
    this.Dejarabajo = Dejarabajo;
    this.ModoSubstationCubo = ModoSubstationCubo;

    this.velocidadInferior = velocidadInferior;
    this.velocidadSuperior = velocidadSuperior;

    this.brazosubsystem = brazosubsystem;
    addRequirements(brazosubsystem);

  }

  @Override
  public void initialize() {
    brazosubsystem.config_motor_eje_inf();
    brazosubsystem.config_motor_eje_sup();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (SafeArm.get()) {

      brazosubsystem.movimiento_brazo_angulo(posicionSafeArm.ejeinferior,
          posicionSafeArm.ejesuperior); // centrode

    } else if (ModoSubstationCono.get()) {

      brazosubsystem.movimiento_brazo_angulo(posicionConodoublesubstation.ejeinferior,
          posicionConodoublesubstation.ejesuperior);// substation cono

    } else if (ModoSubstationCubo.get()) {

      brazosubsystem.movimiento_brazo_angulo(posicionCuboDoubleSub.ejeinferior,
          posicionCuboDoubleSub.ejesuperior);// substation Cubo

    } else if (ReturnHome.get()) {

      brazosubsystem.returnhome(ReturnHome.get()); // Reinicio sensores

    } else if (DejarConoEnmedio.get()) {

      brazosubsystem.movimiento_brazo_angulo(posiciondejarcono.ejeinferior, posiciondejarcono.ejesuperior);/// dejar
                                                                                                           /// cono
    } else if (DejarCubomid.get()) {
      brazosubsystem.movimiento_brazo_angulo(posiciondejarCubo.ejeinferior, posiciondejarCubo.ejesuperior);
    } else if (BotonDesbloqueo.get()) {

      brazosubsystem.ejeinferior(velocidadInferior.get());// Control manual
      brazosubsystem.ejesuperior(velocidadSuperior.get());

    } else if (DejarCuboarriba.get()) {
      brazosubsystem.movimiento_brazo_angulo(posiciondejarCuboArriba.ejeinferior, posiciondejarCuboArriba.ejesuperior);

    } else if (Dejarabajo.get()) {
      brazosubsystem.movimiento_brazo_angulo(posiciondejarAbajo.ejeinferior, posiciondejarAbajo.ejesuperior);

    } else if (ModoRecogePiso.get()) {

      brazosubsystem.movimiento_brazo_angulo(posicioncubopiso.ejeinferior, posicioncubopiso.ejesuperior); // Posicion
                                                                                                          // agrrar del
                                                                                                          // piso

    } else {
      brazosubsystem.ejeinferior(0);
      brazosubsystem.ejesuperior(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    brazosubsystem.ejeinferior(0);
    brazosubsystem.ejesuperior(0);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
