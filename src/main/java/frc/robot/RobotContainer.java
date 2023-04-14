
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.BrazoCmd;
import frc.robot.commands.DriveTrainCmd;
import frc.robot.commands.GarraCmd;
import frc.robot.commands.Auto.auto_config.Cono_engaged_Y_MB;
import frc.robot.commands.Auto.auto_config.Cono_mobility_CC;
import frc.robot.commands.Auto.auto_config.Cono_mobility_SC;
import frc.robot.commands.Auto.auto_config.Cubo_Mobility_CC;
import frc.robot.commands.Auto.auto_config.Cubo_Mobility_SC;
import frc.robot.commands.Auto.auto_config.Cubo_engaged_y_MB;
import frc.robot.commands.Auto.auto_config.TestPath;
import frc.robot.commands.Auto.auto_config.engaged;
import frc.robot.subsystems.BrazoSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GarraSubsystem;

public class RobotContainer {

  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final static GarraSubsystem garraSusbsytem = new GarraSubsystem();
  public final static BrazoSubsystem brazosubsystem = new BrazoSubsystem();


  public Joystick XboxController_main = new Joystick(0);
  public Joystick XboxController_secondary = new Joystick(1);

  private static SendableChooser<Command> autoChooser = new SendableChooser<>();


  public RobotContainer() {
    Controles();
  }

  private void Controles() {

    configureAuto();

  driveSubsystem.setDefaultCommand(new DriveTrainCmd(driveSubsystem,
  () -> (XboxController_main.getRawAxis(3) - XboxController_main.getRawAxis(2)), // Velocidad
  () -> -XboxController_main.getRawAxis(0), // giro
  () -> XboxController_main.getRawButton(2), // boton autoapuntado limelight
  () -> XboxController_main.getRawButton(3), // boton balanceo
  () -> XboxController_main.getRawButton(1),// boton apuntado substation
  () -> XboxController_main.getRawButton(4)//boton robot tieso

  ));

  //new JoystickButton(XboxController_main,5).toggleOnTrue(new GarraCmd(garraSusbsytem, ()->true, ()->false));
  new POVButton(XboxController_main, 90).toggleOnTrue(new GarraCmd(garraSusbsytem, ()->true, ()->false, ()->false));
  new POVButton(XboxController_main, 180).toggleOnTrue(new GarraCmd(garraSusbsytem, ()->false, ()->true, ()->false));
  new POVButton(XboxController_main, 270).toggleOnTrue(new GarraCmd(garraSusbsytem, ()->false, ()->false, ()->true));


  /*
 * 1  BotonDesbloqueo
 * 2  ReturnHome
 * 3  ModoSubstationCono
 * 4  SafeArm
 * 5  DejarConoEnmedio
 * 6  ModoRecogePiso
 * 7  ModoSubstationCubo
 * 8  DejarCubomid
 * 9  DejarCuboarriba
 * 10 Dejarabajo
 */

  //                                                                                                                                                                                                                1           2         3          4         5          6         7           8          9          10
new JoystickButton(XboxController_secondary, 1).toggleOnTrue(new BrazoCmd(brazosubsystem, () -> XboxController_secondary.getRawAxis(5), () -> XboxController_secondary.getRawAxis(1), ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->true));
new JoystickButton(XboxController_secondary, 2).toggleOnTrue(new BrazoCmd(brazosubsystem, () -> XboxController_secondary.getRawAxis(5), () -> XboxController_secondary.getRawAxis(1), ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->true,  ()->false, ()->false));
new JoystickButton(XboxController_secondary, 3).toggleOnTrue(new BrazoCmd(brazosubsystem, () -> XboxController_secondary.getRawAxis(5), () -> XboxController_secondary.getRawAxis(1), ()->false, ()->false, ()->false, ()->false, ()->true,  ()->false, ()->false, ()->false, ()->false, ()->false));
new JoystickButton(XboxController_secondary, 4).toggleOnTrue(new BrazoCmd(brazosubsystem, () -> XboxController_secondary.getRawAxis(5), () -> XboxController_secondary.getRawAxis(1), ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->true,  ()->false));
new JoystickButton(XboxController_secondary, 5).toggleOnTrue(new BrazoCmd(brazosubsystem, () -> XboxController_secondary.getRawAxis(5), () -> XboxController_secondary.getRawAxis(1), ()->false, ()->true,  ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false));
new JoystickButton(XboxController_secondary, 6).toggleOnTrue(new BrazoCmd(brazosubsystem, () -> XboxController_secondary.getRawAxis(5), () -> XboxController_secondary.getRawAxis(1), ()->true,  ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false));
    
     new POVButton(XboxController_secondary, 0).  toggleOnTrue(new BrazoCmd(brazosubsystem, () -> XboxController_secondary.getRawAxis(5), () -> XboxController_secondary.getRawAxis(1),      ()->false, ()->false, ()->false, ()->true,  ()->false, ()->false, ()->false, ()->false, ()->false, ()->false));
     new POVButton(XboxController_secondary, 270).toggleOnTrue(new BrazoCmd(brazosubsystem, () -> XboxController_secondary.getRawAxis(5), () -> XboxController_secondary.getRawAxis(1),      ()->false, ()->false, ()->true,  ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->false));
     new POVButton(XboxController_secondary, 90). toggleOnTrue(new BrazoCmd(brazosubsystem, () -> XboxController_secondary.getRawAxis(5), () -> XboxController_secondary.getRawAxis(1),      ()->false, ()->false, ()->false, ()->false, ()->false, ()->false, ()->true,  ()->false, ()->false, ()->false));
     new POVButton(XboxController_secondary, 180).toggleOnTrue(new BrazoCmd(brazosubsystem, () -> XboxController_secondary.getRawAxis(5), () -> XboxController_secondary.getRawAxis(1),      ()->false, ()->false, ()->false, ()->false, ()->false, ()->true,  ()->false, ()->false, ()->false, ()->false)); 

  }

  public void configureAuto() {

    autoChooser.setDefaultOption("Null autonomous", null);
    autoChooser.addOption("Mobility", new engaged());
    autoChooser.addOption("Cono en mid y engaged", new Cono_engaged_Y_MB());
    autoChooser.addOption("Cono mobility, zona cables", new Cono_mobility_CC());
    autoChooser.addOption("Cono mobility, zona sin cables", new Cono_mobility_SC());

    autoChooser.addOption("Cubo High y engaged", new Cubo_engaged_y_MB());
    autoChooser.addOption("Cubo mobility, zona cables", new Cubo_Mobility_CC());
    autoChooser.addOption("Cubo mobility, zona sin cables", new Cubo_Mobility_SC());

    autoChooser.addOption("Test Path", new TestPath());


    SmartDashboard.putData("Autonomous", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
