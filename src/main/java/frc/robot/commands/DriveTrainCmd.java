
package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveTrainCmd extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private Supplier<Double> funciongiro, funcionVelocidad;
  private Supplier<Boolean> autoapuntado,apuntadoagarrar,balanceo,robotSinmovimiento;

  public DriveTrainCmd(DriveSubsystem driveSubsystem, Supplier<Double> funcionVelocidad, Supplier<Double> funciongiro,
      Supplier<Boolean> autoapuntado, Supplier<Boolean> balanceo, Supplier <Boolean> apuntadoagarrar,Supplier <Boolean> robotSinmovimiento) {

    this.funcionVelocidad = funcionVelocidad;
    this.funciongiro = funciongiro;
    this.autoapuntado = autoapuntado;
    this.balanceo = balanceo;
    this.apuntadoagarrar=apuntadoagarrar;
    this.robotSinmovimiento=robotSinmovimiento;

    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {

    driveSubsystem.resetSensors();
    
    driveSubsystem.configPIDDrivTr();

  }

  @Override
  public void execute() {

    driveSubsystem.CHASIS(funcionVelocidad.get(), funciongiro.get(), autoapuntado.get(), balanceo.get(), apuntadoagarrar.get(), robotSinmovimiento.get());

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
