
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GarraSubsystem;

public class GarraCmd extends CommandBase {

  public final GarraSubsystem garraSubsystem;
  private Supplier<Boolean> state;

  public GarraCmd(GarraSubsystem garraSubsystem, Supplier <Boolean> state) {

this.state=state;
this.garraSubsystem=garraSubsystem;
addRequirements(garraSubsystem);

  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {

garraSubsystem.pistongarrastate(state.get());

  }


  @Override
  public void end(boolean interrupted) {




  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
