package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class PivotIOSim implements PivotIO {

    private SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNEO(1), 40, 20, 0.4, 0, Math.PI / 4, true, 0);
    private PIDController pid = new PIDController(0.0, 0.0, 0.0);
    private boolean closedLoop = false;
    private double ffVolts = 0.0;
    private double appliedVolts = 0.0;
    private final IntakeSimulation intakeSimulation;
    private boolean newIntakeState = false;

    public PivotIOSim(AbstractDriveTrainSimulation driveTrainSimulation) {
        this.intakeSimulation = IntakeSimulation.InTheFrameIntake(
                // Specify the type of game pieces that the intake can collect
                "Note",
                // Specify the drivetrain to which this intake is attached
                driveTrainSimulation,
                // Specify width of the intake
                Units.Meters.of(0.7),
                // The intake is mounted on the back side of the chassis
                IntakeSimulation.IntakeSide.FRONT,
                // The intake can hold up to 1 note
                1);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngleRads()) + ffVolts, -12.0, 12.0);
            sim.setInputVoltage(appliedVolts);
        }

        sim.update(0.02);

        inputs.positionRad = sim.getAngleRads();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
        inputs.intakeState = newIntakeState;
        inputs.gamePiecesCollected = intakeSimulation.getGamePiecesAmount();
    }

    @Override
    public void setVoltage(double volts) {
        closedLoop = false;
        appliedVolts = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void seekPosition(double position) {
        closedLoop = true;
        pid.setSetpoint(position);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }

    @Override // Defined by IntakeIO
    public void setRunning(boolean runIntake) {
        if (runIntake)
            intakeSimulation
                    .startIntake(); // Extends the intake out from the chassis frame and starts detecting contacts with
        // game pieces
        else
            intakeSimulation
                    .stopIntake(); // Retracts the intake into the chassis frame, disabling game piece collection
    }

    @Override
    public void setIntakeState(boolean state) {
        newIntakeState = state;
    }
}
