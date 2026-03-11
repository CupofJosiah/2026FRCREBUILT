package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    TalonFX shooter = new TalonFX(12);
    TalonFX follower = new TalonFX(15);

    private final Slot0Configs shooterRPSControl = new Slot0Configs();

    private final VelocityVoltage valVolt;

    BangBangController controller = new BangBangController();
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, Constants.ShooterConstants.shooterKV);
    

    public ShooterSubsystem() {
        follower.setControl(new Follower(12, MotorAlignmentValue.Opposed));

    shooterRPSControl.withKV(0.11965);
    shooterRPSControl.withKS(0.3422);
    shooterRPSControl.withKP(0);

    shooter.getConfigurator().apply(shooterRPSControl);
    follower.getConfigurator().apply(shooterRPSControl);

    valVolt = new VelocityVoltage(0);
    }

    public void runVelocity(double rpm) {

        // Controls a motor with the output of the BangBang controller and a feedforward
        // Shrinks the feedforward slightly to avoid overspeeding the shooter
                // Shooter.setVoltage(controller.calculate(Shooter.getVelocity().getValue().in(Units.RPM), rpm) * 12.0 + 0.9 * (rpm / 60 * Constants.ShooterConstants.shooterKV));
   shooter.setControl(valVolt.withVelocity(RPM.of(rpm)));
            }

    public double getVelocity(){
        return shooter.getVelocity().getValue().in(Units.RPM);
    }
}