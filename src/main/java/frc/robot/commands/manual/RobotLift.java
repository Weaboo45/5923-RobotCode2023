package frc.robot.commands.manual;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ScissorLift;


public class RobotLift extends CommandBase {
    private ScissorLift lift;
    private Supplier<Boolean> up, down;

    public RobotLift(ScissorLift arm, Supplier<Boolean> up, Supplier<Boolean> down){
        addRequirements(arm);
        this.lift = arm;
        this.up = up;
        this.down = down;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if(up.get()) {lift.lift(.75); }
        if(up.get() == false ){lift.lift(0);} }

    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
    return false;
    }
}
