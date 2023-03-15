
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BrazoSubsystem;

public class BrazoCmd extends CommandBase {

  private final BrazoSubsystem brazosubsystem;
  private Supplier<Double> velocidadInferior,velocidadSuperior;

  public BrazoCmd(BrazoSubsystem brazosubsystem, Supplier <Double> velocidadInferior, Supplier <Double> velocidadSuperior) {

    this.velocidadInferior=velocidadInferior;
    this.velocidadSuperior=velocidadSuperior;

this.brazosubsystem=brazosubsystem;
addRequirements(brazosubsystem);

  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

brazosubsystem.ejeinferior(velocidadInferior.get());
brazosubsystem.ejesuperior(velocidadSuperior.get());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
