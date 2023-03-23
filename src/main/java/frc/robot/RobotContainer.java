
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

  public RobotContainer() {
    Controles();
  }

  private void Controles() {

    configureAuto();

    driveSubsystem.setDefaultCommand(new DriveTrainCmd(driveSubsystem,
        () -> -XboxController_main.getRawAxis(1),
        () -> (XboxController_main.getRawAxis(3) - XboxController_main.getRawAxis(2)),
        () -> XboxController_main.getRawButton(2),
        () -> XboxController_main.getRawButton(3),
        () -> XboxController_main.getRawButton(1)));

    garraSusbsytem.setDefaultCommand(new GarraCmd(garraSusbsytem, () -> XboxController_main.getRawButtonPressed(5),
        () -> XboxController_main.getRawButtonPressed(6)));

    compresorSubsystem.setDefaultCommand(new CompresorCmd(compresorSubsystem, false));

    brazosubsystem.setDefaultCommand(new BrazoCmd(brazosubsystem,
        () -> XboxController_secondary.getRawAxis(5),
        () -> XboxController_secondary.getRawAxis(1),
        () -> XboxController_secondary.getRawButton(6),
        () -> XboxController_secondary.getRawButton(4),
        () -> XboxController_secondary.getRawButton(2),
        () -> XboxController_secondary.getRawButton(1),
        () -> XboxController_secondary.getRawButton(3)));

    compuertasubsystem
        .setDefaultCommand(new CompuertaCmd(compuertasubsystem,
            () -> XboxController_secondary.getRawButton(5)));

    new JoystickButton(XboxController_main, 7).toggleOnTrue(new CompresorCmd(compresorSubsystem, true));
  }

  public void configureAuto() {

    autoChooser.setDefaultOption("Auto test 1", new mobility());
    autoChooser.addOption("ONE_GP_balanceo", new ONE_GP_balanceo());
    autoChooser.addOption("One_GP_mobility", new ONE_GP_mobility());
    SmartDashboard.putData("Autonomous", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
