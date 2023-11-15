package frc.robot;

import static frc.robot.Controls.ControlConstants.*;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmWristSetTargetCommand;
import frc.robot.commands.IntakeCommandWrist;
import frc.robot.commands.SetArmHeightPreset;
import frc.robot.commands.TimedIntakeCommand;
import frc.robot.subsystems.ArmSubsystem.Height;

public class Controls {
	public static class ControlConstants {
		public static final int CODRIVER_CONTROLLER_PORT = 0;

		
	}

	//private final CommandXboxController driveController;
	private final CommandXboxController armController;

	// Drivebase

	


	private final Subsystems s;

	public Controls(Subsystems s) {
		armController = new CommandXboxController(CODRIVER_CONTROLLER_PORT);
		this.s = s;


		bindArmControls();

		/* below is an example
		if (Subsystems.SubsystemConstants.INTAKE_ENABLED) {
			bindIntakeControls();
		}
		if (Subsystems.SubsystemConstants.LED_ENABLED) {
			bindLEDControls();
		}
		if (Subsystems.SubsystemConstants.ARM_ENABLED) {
			bindArmControls();
		}
		*/
	}

	public void bindDrivebaseControls() {
		
		
	}

	public void bindArmControls() {
		armController.povDown().onTrue(new SequentialCommandGroup(new ArmWristSetTargetCommand(s.arm,0.063, 0.65), new IntakeCommandWrist(s.intake, s.arm)));
		armController.leftBumper().onTrue(new InstantCommand(() -> s.arm.goToHeight()));

		armController.a().onTrue(new SetArmHeightPreset(s.arm, Height.LOW));
		armController.x().onTrue(new SetArmHeightPreset(s.arm, Height.MID));
		armController.y().onTrue(new SetArmHeightPreset(s.arm, Height.HIGH));
		armController.b().onTrue(new ArmWristSetTargetCommand(s.arm,0.37, 0.8));

		armController.povUp().onTrue(new SequentialCommandGroup(new ArmWristSetTargetCommand(s.arm,0.22, 0.57), new IntakeCommandWrist(s.intake, s.arm)));

		
	
		
		//Need to turn off intake 
		
		armController.rightTrigger().onTrue(new TimedIntakeCommand(s.intake, -0.8));
		armController.leftTrigger().onTrue(new TimedIntakeCommand(s.intake, -0.3));
	}



	/* AN EXSMPLE (so not deleted yet)
	public void bindIntakeControls() {
		CommandScheduler.getInstance()
				.setDefaultCommand(s.intakeSubsystem, new IntakeDefaultCommand(s.intakeSubsystem));

		// Drive Buttons

		driveIntakeInButton.onTrue(new IntakeSetInCommand(s.intakeSubsystem));
		driveIntakeOutButton.onTrue(new IntakeSetOutCommand(s.intakeSubsystem));
		driveIntakeFastOutButton.onTrue(new IntakeSetFastOutCommand(s.intakeSubsystem));

		// Codrive buttons
		codriveIntakeInButton.onTrue(new IntakeSetInCommand(s.intakeSubsystem));
		codriveIntakeOutButton.onTrue(new IntakeSetOutCommand(s.intakeSubsystem));
		// if (Subsystems.SubsystemConstants.LED_ENABLED) {
		// 	codriveIntakeOutButton.onTrue(new IntakeOutCommand(s.intakeSubsystem, s.ledSubsystem));
		// } else {
		// 	codriveIntakeOutButton.onTrue(new IntakeSetOutCommand(s.intakeSubsystem));
		// }
		// codriveIntakeStopButton.onTrue(new IntakeSetStopCommand(s.intakeSubsystem));
	}

	*/

	
}