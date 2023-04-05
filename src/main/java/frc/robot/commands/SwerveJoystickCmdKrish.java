package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystemKrish;

public class SwerveJoystickCmdKrish extends CommandBase{

    private final Supplier<Double> power;
    private final SwerveSubsystemKrish swerve;

    public SwerveJoystickCmdKrish(Supplier<Double> power, SwerveSubsystemKrish swerve){
        this.power = power;
        this.swerve = swerve;

        addRequirements(swerve);
    }



    @Override
    public void initialize(){
        System.out.println("It started");

    }

    @Override
    public void execute(){
        double realTimePower = power.get();
        swerve.setDrivePower(realTimePower);

        
        
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