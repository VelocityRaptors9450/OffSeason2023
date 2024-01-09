package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmWristSetTargetCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommandWrist;
import frc.robot.commands.SetArmHeightPreset;
import frc.robot.commands.TimedIntakeCommand;
import frc.robot.subsystems.ArmSubsystem.Height;

// Manage pathplanner auto stuff
public class Autos {

  public Subsystems subsystems;

  public Autos(Subsystems subsystems) {
    this.subsystems = subsystems;
  }

  /**
   * Returns a full autonomous path from the given path name, velocity, and acceleration constraints
   * @param path The name of the path to load (Specified in the gui)
   * @param maxVelocity The maximum velocity the path should achieve (m/s)
   * @param maxAcceleration The maximum acceleration the path should achieve (m/s)
   * @return Autonomous command
   */
  public Command getAuto(String path, double maxVelocity, double maxAcceleration) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, new PathConstraints(maxVelocity, maxAcceleration));
    return getAutoBuilder(getEventMap()).fullAuto(pathGroup);
  }

  // configure event map
  // the event map maps strings to commands
  // these commands can be run by creating an event marker in the gui with the corresponding string as its name
  // if the event is marked as a stop point in the gui, the robot will wait until the command finishes before continuing
  private HashMap<String, Command> getEventMap() {
    HashMap<String, Command> eventMap = new HashMap<String, Command>();

    // Set up commands
    Command scoreMid = new SetArmHeightPreset(subsystems.arm, Height.MID)
        .andThen(new InstantCommand(() -> subsystems.arm.goToHeight()));

    Command scoreHigh = new SetArmHeightPreset(subsystems.arm, Height.HIGH)
        .andThen(new InstantCommand(() -> subsystems.arm.goToHeight()));

    Command intake = new ArmWristSetTargetCommand(subsystems.arm, 0.063, 0.43)
        .andThen(new IntakeCommandWrist(subsystems.intake, subsystems.arm));

    Command spin;// = new DriveCommand(subsystems.drivebaseSubsystem, () -> -0.2, () -> 0, () -> 0, () -> 0).withTimeout(2);

    Command wristUp = new InstantCommand(() -> subsystems.arm.setWristGoal(0.65));

    Command resetIMU = new InstantCommand(() -> subsystems.drivebaseSubsystem.resetGyroAngle());

    Command score = new TimedIntakeCommand(subsystems.intake, -0.5)
        .andThen(new ArmWristSetTargetCommand(subsystems.arm, 0.37, 0.6));

    Command wait = new WaitCommand(2);

    // Add commands to event map
    eventMap.put("ArmMid", scoreMid);
    eventMap.put("ArmHigh", scoreHigh);
    eventMap.put("Intake", intake);
    //eventMap.put("Creep", spin);
    eventMap.put("Wrist Up", wristUp);
    eventMap.put("ResetIMU", resetIMU);
    eventMap.put("Score", score);
    eventMap.put("Wait", wait);

    return eventMap;
  }

  // returns an autobuilder object to use when constructing an auto
  // specify x/y/rotation pid constants here
  public SwerveAutoBuilder getAutoBuilder(HashMap<String, Command> eventMap) {
		if (subsystems.drivebaseSubsystem != null) {
			return new SwerveAutoBuilder(
					subsystems.drivebaseSubsystem::getPose, // Pose2d supplier
					subsystems.drivebaseSubsystem
							::resetPose, // Pose2d consumer, used to reset odometry at the beginning of
					// auto
					subsystems.drivebaseSubsystem.getKinematics(), // SwerveDriveKinematics
					new PIDConstants(
							3, 0.0,
							0.1), // PID constants to correct for translation error (used to create the X and
					// Y
					// PID controllers)
					new PIDConstants(
						  3, 0.0,
							0.3), // PID constants to correct for rotation error (used to create the rotation
					// controller)
					subsystems.drivebaseSubsystem
							::drive, // Module states consumer used to output to the drive subsystem
					eventMap,
					true, // Should the path be automatically mirrored depending on alliance color.
					// Optional, defaults to true
					subsystems
							.drivebaseSubsystem // The drive subsystem. Used to properly set the requirements
					// of
					// path following commands
					);
		} else {
			return null;
		}
	}
  
}
