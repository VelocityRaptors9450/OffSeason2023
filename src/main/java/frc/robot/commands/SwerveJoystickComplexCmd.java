package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystemKrish;

public class SwerveJoystickComplexCmd extends CommandBase{

    private final Supplier<Double> drivePower;
    private final Supplier<Double> turnY;
    private final Supplier<Double> turnX;
    private final Supplier<Boolean> isAPressed;
    private boolean turning = false;
    private double startTime = 0;
    private Timer time;

    private final SwerveSubsystemKrish swerve;

    public SwerveJoystickComplexCmd(Supplier<Double> drivePower, Supplier<Double> turnY, Supplier<Double> turnX, Supplier<Boolean> isAPressed, SwerveSubsystemKrish swerve){
        this.drivePower = drivePower;
        this.turnY = turnY;
        this.turnX = turnX;
        this.isAPressed = isAPressed;

        this.swerve = swerve;
        time = new Timer();
        time.start();
        time.reset();

        addRequirements(swerve);
    }



    @Override
    public void initialize(){
       

    }

    @Override
    public void execute(){
        double realTimeDrivePower =  -1 * drivePower.get() / 3;
        double realTimeTurnPowerY =  1 * turnY.get() / 3;
        double realTimeTurnPowerX =  -1 * turnX.get() / 3;
        boolean aButton = isAPressed.get();

        if(aButton){
            turning = true;
            startTime = time.get();
        }

        
        if(Math.abs(realTimeDrivePower) > 0.05){
            swerve.setDrivePower(realTimeDrivePower);

        }else{
            swerve.setDrivePower(0);
        }



        if(turning){
            swerve.pid(-Math.PI / 4, 5 * Math.PI / 4, Math.PI / 4, 3 * Math.PI / 4, 0.4);

            if(time.get() - startTime > 1){
                turning = false;
            }
            
        }else if(Math.abs(realTimeTurnPowerX) > 0.05 || Math.abs(realTimeTurnPowerY) > 0.05){

            double angle = Math.atan2(realTimeTurnPowerY, realTimeTurnPowerX) - (Math.PI / 2);

            swerve.pid(angle, angle, angle, angle, 0.4);

            //System.out.println(swerve.getTurningEncoderFL());
        }else{
            swerve.setTurningPower(0, 0, 0, 0);
        }

        
        
        
    }

    @Override
    public void end(boolean interrupted){
       
        
    }

    @Override
    public boolean isFinished(){
        return false;   
    }
}