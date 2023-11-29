package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ScissorLift extends SubsystemBase {
    private WPI_VictorSPX liftMotor;

    private ShuffleboardTab tab;

    public ScissorLift(ShuffleboardTab tab){

        liftMotor = new WPI_VictorSPX(LIFT_MOTOR);
        liftMotor.setNeutralMode(NeutralMode.Brake);
        this.tab = tab;
        configureShuffleboardData();
    }

    private void configureShuffleboardData() {
        
    }

    @Override
    public void periodic() {
    }

    public void lift(double speed){
        liftMotor.set(speed);
    }
}
