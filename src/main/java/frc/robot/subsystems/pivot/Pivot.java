// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private final SysIdRoutine sysId;
    private Pose3d pivotPose = new Pose3d();
    double setPos = 0.0;

    /** Creates a new Flywheel. */
    public Pivot(PivotIO io) {
        this.io = io;

        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
                io.configurePID(1.0, 0.0, 0.0);
                break;
            case SIM:
                io.configurePID(0.5, 0.0, 0.0);
                break;
            default:
                break;
        }

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        var pivotSetPose = new Pose3d(-0.17, 0, 0.18, new Rotation3d(0, -setPos, 0));

        pivotPose = new Pose3d(0, 0, 0, new Rotation3d(0, inputs.positionRad, 0));
        Logger.recordOutput("Pivot/Position", pivotPose);
        Logger.recordOutput("Pivot/SetPosition", pivotSetPose);

        io.setRunning(inputs.intakeState);
    }

    public void setIntakeState(boolean state) {
        io.setIntakeState(state);
    }

    /** Run open loop at the specified voltage. */
    public void runVolts(double volts) {
        io.setVoltage(volts);
    }

    /** Run closed loop position PID. */
    public void seekPosition(double position) {
        io.seekPosition(position);
        // Log flywheel setpoint
        setPos = position;
        Logger.recordOutput("Pivot/SetpointPos", position);
    }

    /** Stops the flywheel. */
    public void stop() {
        io.stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    /** Returns the current velocity in RPM. */
    @AutoLogOutput
    public double getVelocityRPM() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    }

    /** Returns the current velocity in radians per second. */
    public double getCharacterizationVelocity() {
        return inputs.velocityRadPerSec;
    }
}
