package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeSubsystem extends SubsystemBase{
    public CANSparkMax intake = new CANSparkMax(Constants.intakeId, MotorType.kBrushless);

    private Timer t = new Timer();
    private Timer g = new Timer();

    public static enum IntakeStep {
        INITIAL_RAMP_UP,
        INITIAL, // rampUpToggle boolean equivalent
        SLOW_INTAKE,
        IS_OUTPUTTING,
        DONE_OUTPUTTING,
    }

    private IntakeStep intakeStep = IntakeStep.INITIAL;

    private static double oldpos = 0;
    private static boolean temp = true;
    private static boolean isOutput = false;
    private static boolean initial = true;
    private static boolean rampUPToggle = true;
    public static double velocity = 10;

    public IntakeSubsystem(){
        

        intake.restoreFactoryDefaults();
        //Might need this line
        intake.setIdleMode(IdleMode.kBrake);
        
    }

    public double rampUp(double time, double powerTo) {
        return 2 * powerTo / (1 + Math.pow(Math.E, -9*(time))) - 0.4;

    }

    public void setIntakePower(double speed) {
        intake.set(speed);
    }

    public void stopIntakeMotor() {
        intake.stopMotor();
    }

    public void resetIntakeVars() {
        temp = true;
        isOutput = false;
        initial = true;
        rampUPToggle = true;
        velocity = 0;
    }
    

    public void intake(double power) {
        
        double changeInPos = Math.abs(intake.getEncoder().getPosition()) - oldpos;
        velocity = changeInPos / 0.02;
        oldpos = Math.abs(intake.getEncoder().getPosition());
        // approximately 19 revolutions / second
        
        if (velocity < 4500 && initial) {
            if (rampUPToggle) {
                // assigns current state
                intakeStep = IntakeStep.INITIAL_RAMP_UP;
                t.restart();
                rampUPToggle = false;
            }
            

            if (rampUp(t.get(), 0.4) > 0.4) {
                intake.set(power);
            } else {
                //System.out.println("------------------- \n RAMPING UP \n ---------------------------");
                intake.set(rampUp(t.get(), 0.4));
                
            }
            
        } else if (!isOutput && velocity > 3) { // intake 
            //System.out.println("MADE ITTTTTTTTTTTTTTTTTTTTTT");
            intakeStep = IntakeStep.INITIAL;
            initial = false;
            intake.set(power);
        } else if (!isOutput){
            if (temp) {
                g.reset(); 

                temp = false;
                //System.out.println("------------------- \n SLOW SPEED \n ---------------------------");
                intakeStep = IntakeStep.SLOW_INTAKE;
                intake.set(0.04);
                
            }
            
        }

       
    }


}