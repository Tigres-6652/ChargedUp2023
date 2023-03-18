
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BrazoSubsystem;

public class BrazoCmd extends CommandBase {

  private final BrazoSubsystem brazosubsystem;
  private Supplier<Double> velocidadInferior, velocidadSuperior;
  private Supplier<Boolean> BotonArriba, BotonDerecha, BotonAbajo, BotonIzquierda;

  public BrazoCmd(BrazoSubsystem brazosubsystem, Supplier<Double> velocidadInferior, Supplier<Double> velocidadSuperior,
      Supplier<Boolean> BotonArriba, Supplier<Boolean> BotonDerecha, Supplier<Boolean> BotonAbajo,
      Supplier<Boolean> BotonIzquierda) {

    this.BotonArriba = BotonArriba;
    this.BotonDerecha = BotonDerecha;
    this.BotonAbajo = BotonAbajo;
    this.BotonIzquierda = BotonIzquierda;
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

    } else if (BotonDerecha.get()) {
      
brazosubsystem.movimiento_brazo_angulo(39,-84);// substation

    } else if (BotonArriba.get()) {

      brazosubsystem.returnhome(BotonArriba.get());


    } else if (BotonIzquierda.get()) {

      brazosubsystem.movimiento_brazo_angulo(73, -146);///dejar cono


    } else {

      brazosubsystem.ejeinferior(velocidadInferior.get());
      brazosubsystem.ejesuperior(velocidadSuperior.get());

    }

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
