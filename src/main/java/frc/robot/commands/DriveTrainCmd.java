
package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainCmd extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private Supplier<Double> funciongiro, funcionVelocidad;
  private Supplier<Boolean> autoapuntado,apuntadoagarrar,balanceo;

  public DriveTrainCmd(DriveSubsystem driveSubsystem, Supplier<Double> funcionVelocidad, Supplier<Double> funciongiro,
      Supplier<Boolean> autoapuntado, Supplier<Boolean> balanceo, Supplier <Boolean> apuntadoagarrar) {

    this.funcionVelocidad = funcionVelocidad;
    this.funciongiro = funciongiro;
    this.autoapuntado = autoapuntado;
    this.balanceo = balanceo;
    this.apuntadoagarrar=apuntadoagarrar;

    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {

    driveSubsystem.resetTalons();

  }

  @Override
  public void execute() {

    driveSubsystem.CHASIS(funcionVelocidad.get(), funciongiro.get(), autoapuntado.get(), balanceo.get(), apuntadoagarrar.get());

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
