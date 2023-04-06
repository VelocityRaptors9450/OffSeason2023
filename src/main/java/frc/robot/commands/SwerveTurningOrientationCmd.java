package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystemKrish;

public class SwerveTurningOrientationCmd extends CommandBase{

    private final SwerveSubsystemKrish swerve;
    private final boolean toTurn;

    public SwerveTurningOrientationCmd(SwerveSubsystemKrish swerve, boolean toTurn){
        this.swerve = swerve;
        this.toTurn = toTurn;

        addRequirements(swerve);

        
    }



    @Override
    public void initialize(){
        System.out.println("It started");

    }

    @Override
    public void execute(){
        swerve.changeOrienation(toTurn);
        
        
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("It ended");
        
    }

    @Override
    public boolean isFinished(){
        return false;   
    }
}