
package frc.robot.commands;

import java.util.function.Supplier;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GarraSubsystem;

public class GarraCmd extends CommandBase {

  public final GarraSubsystem garraSubsystem;
  private Supplier<Boolean> abre, cierra;
  boolean conAbre;
  boolean conCierra;
  int entrada=0;

  public GarraCmd(GarraSubsystem garraSubsystem, Supplier <Boolean> abre,Supplier <Boolean> cierra) {

this.abre=abre;
this.cierra=cierra;
this.garraSubsystem=garraSubsystem;
addRequirements(garraSubsystem);

  }


  @Override
  public void initialize() {
  }


  @Override
  public void execute() {



garraSubsystem.pistongarrastate(abre.get(),cierra.get());
SmartDashboard.putNumber("Entrada GarraCmd", entrada);
if(abre.get()){
  conAbre=true;
  entrada = 1;
}
if(cierra.get()){
  entrada = 2;
  conCierra=true;
}

  }


  @Override
  public void end(boolean interrupted) {




  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
   if(DriverStation.isAutonomousEnabled()){
      if(conAbre && !conCierra){
        return true;
      }else if(!conAbre && !conCierra){
        return false;
      }else if(!conAbre && conCierra){
        return true;
      }else {
        return true;
      }
}else{
  return false;
}
}
}

