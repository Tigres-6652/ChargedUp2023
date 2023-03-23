
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.posicionesbrazo.posicioncentrodemasaabajoydentroframe;
import frc.robot.Constants.posicionesbrazo.posiciondejarcono;
import frc.robot.Constants.posicionesbrazo.posiciondoublesubstation;
import frc.robot.subsystems.BrazoSubsystem;

public class BrazoCmd extends CommandBase {

  private final BrazoSubsystem brazosubsystem;
  private Supplier<Double> velocidadInferior, velocidadSuperior;
  private Supplier<Boolean> BotonArriba, BotonDerecha, BotonAbajo, BotonIzquierda, BotonDesbloqueo;

  public BrazoCmd(BrazoSubsystem brazosubsystem, Supplier<Double> velocidadInferior, Supplier<Double> velocidadSuperior,
      Supplier<Boolean> BotonDesbloqueo, Supplier<Boolean> BotonArriba, Supplier<Boolean> BotonDerecha,
      Supplier<Boolean> BotonAbajo,
      Supplier<Boolean> BotonIzquierda) {

    this.BotonArriba = BotonArriba;
    this.BotonDerecha = BotonDerecha;
    this.BotonAbajo = BotonAbajo;
    this.BotonIzquierda = BotonIzquierda;
    this.BotonDesbloqueo = BotonDesbloqueo;

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

    if (BotonAbajo.get()) {

      brazosubsystem.movimiento_brazo_angulo(posicioncentrodemasaabajoydentroframe.ejeinferior,
          posicioncentrodemasaabajoydentroframe.ejesuperior); // centrode

    } else if (BotonDerecha.get()) {

      brazosubsystem.movimiento_brazo_angulo(posiciondoublesubstation.ejeinferior,
          posiciondoublesubstation.ejesuperior);// substation

    } else if (BotonArriba.get()) {

      brazosubsystem.returnhome(BotonArriba.get());

    } else if (BotonIzquierda.get()) {

      brazosubsystem.movimiento_brazo_angulo(posiciondejarcono.ejeinferior, posiciondejarcono.ejesuperior);/// dejar
                                                                                                           /// cono

    } else if (BotonDesbloqueo.get()) {

      brazosubsystem.ejeinferior(velocidadInferior.get());
      brazosubsystem.ejesuperior(velocidadSuperior.get());

    } else {
      brazosubsystem.ejeinferior(0);
      brazosubsystem.ejesuperior(0);
    }
    SmartDashboard.putBoolean("desbloqueo", BotonDesbloqueo.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
