
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BrazoCmd;
import frc.robot.commands.CompuertaCmd;
import frc.robot.commands.DriveTrainCmd;
import frc.robot.commands.GarraCmd;
import frc.robot.subsystems.BrazoSubsystem;
import frc.robot.subsystems.CompuertaSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GarraSubsystem;

public class RobotContainer {

  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final static GarraSubsystem garraSusbsytem = new GarraSubsystem();
  public final static BrazoSubsystem brazosubsystem = new BrazoSubsystem();
  public final static CompuertaSubsystem compuertasubsystem = new CompuertaSubsystem();

  public Joystick XboxController_main = new Joystick(0);
  public Joystick XboxController_secondary = new Joystick(1);

  public RobotContainer() {

    Controles1();

  }

  private void Controles1() {
    driveSubsystem.setDefaultCommand(new DriveTrainCmd(driveSubsystem, () -> XboxController_main.getRawAxis(3) -
        XboxController_main.getRawAxis(2), () -> XboxController_main.getRawAxis(0)));

    garraSusbsytem.setDefaultCommand(new GarraCmd(garraSusbsytem, () -> false));

    new JoystickButton(XboxController_secondary, 1).toggleOnTrue(new GarraCmd(garraSusbsytem, () -> true));

    brazosubsystem.setDefaultCommand(new BrazoCmd(brazosubsystem, () -> XboxController_main.getRawAxis(5),
        () -> XboxController_main.getRawAxis(1), () -> XboxController_secondary.getRawButton(1),
        () -> XboxController_secondary.getRawButton(2), () -> XboxController_secondary.getRawButton(3),
        () -> XboxController_secondary.getRawButton(4)));
    compuertasubsystem
        .setDefaultCommand(new CompuertaCmd(compuertasubsystem, () -> XboxController_secondary.getRawButton(2)));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
