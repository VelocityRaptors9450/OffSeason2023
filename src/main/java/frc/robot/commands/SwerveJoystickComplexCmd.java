package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystemKrish;

public class SwerveJoystickComplexCmd extends CommandBase{

    private final Supplier<Double> drivePower;
    private final Supplier<Double> turnPower;

    private final SwerveSubsystemKrish swerve;

    public SwerveJoystickComplexCmd(Supplier<Double> drivePower, Supplier<Double> turnPower, SwerveSubsystemKrish swerve){
        this.drivePower = drivePower;
        this.turnPower = turnPower;
        this.swerve = swerve;

        addRequirements(swerve);
    }



    @Override
    public void initialize(){
       

    }

    @Override
    public void execute(){
        double realTimeDrivePower =  -1 * drivePower.get() / 3;
        double realTimeTurnPower =  1 * turnPower.get() / 3;
        swerve.setDrivePower(realTimeDrivePower);
        swerve.setTurningPower(realTimeTurnPower, 0, 0, 0);

        System.out.println(swerve.getTurningEncoderFL());

        
        
    }

    @Override
    public void end(boolean interrupted){
       
        
    }

    @Override
    public boolean isFinished(){
        return false;   
    }
}