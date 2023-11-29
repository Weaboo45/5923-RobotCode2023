package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class TopArm extends SubsystemBase {
    private WPI_TalonSRX topArmTalon;
    private WPI_VictorSPX topArmIntakeMotor, topArmVictor, liftMotor;
    private DigitalInput topChannelA = new DigitalInput(TOP_ENCODER_PORT_A);
    private DigitalInput topChannelB = new DigitalInput(TOP_ENCODER_PORT_B);
    public Encoder topEncoder = new Encoder(topChannelA, topChannelB, false, EncodingType.k4X);
    private ShuffleboardLayout layout;
    private double setpoint;

    private ShuffleboardTab tab;

    public TopArm(ShuffleboardTab tab){
        topArmTalon = new WPI_TalonSRX(TOP_ARM_TALON);
        topArmTalon.setNeutralMode(NeutralMode.Brake);

        topArmIntakeMotor = new WPI_VictorSPX(TOP_ARM_INTAKE);
        topArmVictor = new WPI_VictorSPX(TOP_ARM_VICTOR);
        topArmIntakeMotor.setNeutralMode(NeutralMode.Brake);

        liftMotor = new WPI_VictorSPX(LIFT_MOTOR);

        topArmVictor.setInverted(true);
        topArmVictor.follow(topArmTalon);

        this.tab = tab;
        configureShuffleboardData();

        topEncoder.setDistancePerPulse(TOP_ARM_DISTANCE_PER_PULSE);
        topEncoder.setMinRate(0.1);

        // ADD IN AMP LIMITS
    }

    private void configureShuffleboardData() {
        layout = tab.getLayout("Top arm encoder", BuiltInLayouts.kList).withPosition(0, 3);
        layout.add(this);

        layout.addNumber("Top Arm Encoder", () -> getTopEncoderPosition());
        layout.add("Reset encoder", new InstantCommand(() -> topEncoder.reset()));
        //layout.addNumber("Current top arm setpoint", () -> setpoint);

        layout.addNumber("Top Arm Talon Volts", () -> topArmTalon.getMotorOutputVoltage());
        layout.addNumber("Top Arm Intake Volts", () -> topArmIntakeMotor.getMotorOutputVoltage());
        layout.addNumber("Top Arm Victor Volts", () -> topArmVictor.getMotorOutputVoltage());
        layout.addNumber("Break motor Volts", () -> liftMotor.getMotorOutputVoltage());

    }

    public void feedCurrentSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public void periodic() {
    }

    public void releaseObject(double speed) {
        topArmIntakeMotor.set(speed);
    }

    //Upper intake arm
    public void move(double speed) {
        topArmTalon.set(speed);
        //layout.addNumber("Top Arm Voltage", () -> topArmTalon.getBusVoltage());
    }

    public double getTopEncoderPosition() {
        return topEncoder.get();
    }
    
    //lifting mechanism on robot
    public void lift(double speed) {
        liftMotor.set(speed);
    }
}
