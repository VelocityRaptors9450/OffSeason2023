package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystemKrish;

public class SwerveJoystickCmdKrish extends CommandBase{

    private final Supplier<Double> drivePower;
    private final Supplier<Double> turnPower;

    private final SwerveSubsystemKrish swerve;

    public SwerveJoystickCmdKrish(Supplier<Double> drivePower, Supplier<Double> turnPower, SwerveSubsystemKrish swerve){
        this.drivePower = drivePower;
        this.turnPower = turnPower;
        this.swerve = swerve;

        addRequirements(swerve);
    }



    @Override
    public void initialize(){
        System.out.println("It started");

    }

    @Override
    public void execute(){
        double realTimeDrivePower =  -1 * drivePower.get() / 3;
        double realTimeTurnPower =  1 * turnPower.get() / 3;
        swerve.setDrivePower(realTimeDrivePower);
        swerve.setTurningPower(realTimeTurnPower, realTimeTurnPower, realTimeTurnPower, realTimeTurnPower);

        //System.out.println(swerve.getTurningEncoderFL());

        
        
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