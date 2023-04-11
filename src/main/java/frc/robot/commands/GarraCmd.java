
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GarraSubsystem;

public class GarraCmd extends CommandBase {

  public final GarraSubsystem garraSubsystem;
  private Supplier<Boolean> lanzarcubo, lanzarcono, agarrargp;

  public GarraCmd(GarraSubsystem garraSubsystem, Supplier <Boolean> lanzarcubo,Supplier <Boolean> agarrargp, Supplier<Boolean> lanzarcono) {

this.lanzarcubo=lanzarcubo;
this.agarrargp=agarrargp;
this.lanzarcono=lanzarcono;
this.garraSubsystem=garraSubsystem;
addRequirements(garraSubsystem);

  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {



garraSubsystem.stategarra(lanzarcubo.get(), lanzarcono.get(), agarrargp.get());
  }


  @Override
  public void end(boolean interrupted) {

    garraSubsystem.stategarra(false, false, false);



  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
