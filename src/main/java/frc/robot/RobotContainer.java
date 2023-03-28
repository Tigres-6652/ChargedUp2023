
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BrazoCmd;
import frc.robot.commands.CompresorCmd;
import frc.robot.commands.CompuertaCmd;
import frc.robot.commands.DriveTrainCmd;
import frc.robot.commands.GarraCmd;
import frc.robot.commands.Auto.auto_config.ONE_GP_balanceo;
import frc.robot.commands.Auto.auto_config.ONE_GP_mobility;
import frc.robot.commands.Auto.auto_config.UNA_GP;
import frc.robot.commands.Auto.auto_config.mobility;
import frc.robot.subsystems.BrazoSubsystem;
import frc.robot.subsystems.CompresorSubsystem;
import frc.robot.subsystems.CompuertaSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GarraSubsystem;

public class RobotContainer {

  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final static GarraSubsystem garraSusbsytem = new GarraSubsystem();
  public final static BrazoSubsystem brazosubsystem = new BrazoSubsystem();
  public final static CompuertaSubsystem compuertasubsystem = new CompuertaSubsystem();
  public final static CompresorSubsystem compresorSubsystem = new CompresorSubsystem();

  public Joystick XboxController_main = new Joystick(0);
  public Joystick XboxController_secondary = new Joystick(1);

  private static SendableChooser<Command> autoChooser = new SendableChooser<>();

  boolean controles_angel=false;    //Cambiar variable si maneja Angel

  public RobotContainer() {
    Controles();
  }

  private void Controles() {

    configureAuto();
if(controles_angel){
  driveSubsystem.setDefaultCommand(new DriveTrainCmd(driveSubsystem,
  () -> -XboxController_main.getRawAxis(3), // Velocidad
  () -> (XboxController_main.getRawAxis(3) - XboxController_main.getRawAxis(2)), // giro
  () -> XboxController_main.getRawButton(2), // boton autoapuntado
  () -> XboxController_main.getRawButton(3), // boton balanceo
  () -> XboxController_main.getRawButton(1))); // boton autoapuntado

}else{
  driveSubsystem.setDefaultCommand(new DriveTrainCmd(driveSubsystem,
  () -> (XboxController_main.getRawAxis(3) - XboxController_main.getRawAxis(2)), // Velocidad
  () -> XboxController_main.getRawAxis(0), // giro
  () -> XboxController_main.getRawButton(2), // boton autoapuntado limelight
  () -> XboxController_main.getRawButton(3), // boton balanceo
  () -> XboxController_main.getRawButton(1))); // boton apuntado substation0
}


    garraSusbsytem.setDefaultCommand(new GarraCmd(garraSusbsytem,
        () -> XboxController_main.getRawButtonPressed(5), //boton abre
        () -> XboxController_main.getRawButtonPressed(6))); //boton cierra

    compresorSubsystem.setDefaultCommand(new CompresorCmd(compresorSubsystem, false)); //No moverle

    brazosubsystem.setDefaultCommand(new BrazoCmd(brazosubsystem,
        () -> XboxController_secondary.getRawAxis(5),    //control brazo inferior
        () -> XboxController_secondary.getRawAxis(1),    //control brazo superior
        () -> XboxController_secondary.getRawButton(6),//boton desbloqueo_mover joints con joystick
        () -> XboxController_secondary.getRawButton(4),//return home
        () -> XboxController_secondary.getRawButton(2),//substation
        () -> XboxController_secondary.getRawButton(1),//centro de masa_brazo dentro de frame
        () -> XboxController_secondary.getRawButton(3)//dejar cono
        ));

    compuertasubsystem
        .setDefaultCommand(new CompuertaCmd(compuertasubsystem,
            () -> XboxController_secondary.getRawButton(5)));

    new JoystickButton(XboxController_main, 7).toggleOnTrue(new CompresorCmd(compresorSubsystem, true));
  }

  public void configureAuto() {

    autoChooser.setDefaultOption("Mobility", new mobility());
    SmartDashboard.putData("Autonomous", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
