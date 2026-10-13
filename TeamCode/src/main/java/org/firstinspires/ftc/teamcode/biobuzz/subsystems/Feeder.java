package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Intake motor + transfer motor + gate servo.
 *
 *  IDLE   - everything off, gate closed
 *  INTAKE - intake in, transfer slowly stages pieces up against the closed gate
 *  FEED   - gate open, intake + transfer push a piece into the flywheel
 *  FEED_HOLD - gate open, everything stopped (between feed pulses; the roller squeeze holds pieces)
 *  EJECT  - both reversed, gate closed
 *
 * Jam handling: if either motor's current stays above JAM_CURRENT_AMPS for JAM_TIME_S
 * while running forward, both briefly reverse, then resume automatically.
 */
public class Feeder {
    public enum Mode { IDLE, INTAKE, FEED, EJECT, FEED_HOLD }

    public final DcMotorEx intake, transfer;
    public final Servo gate;
    private final HwCache.Power intakeOut, transferOut;
    private final HwCache.Position gateOut;
    private boolean readTransferNext;

    public Mode mode = Mode.IDLE;
    public double intakeAmps, transferAmps;
    public boolean unjamming;
    public int jams;
    public boolean gateOpen;

    /** Tuners only: if not NaN, the gate servo is held here instead. */
    public double gateOverride = Double.NaN;

    private double overCurrentSince = -1;
    private double unjamUntil = -1;
    private double gateOpenedAt = -1;

    public Feeder(HardwareMap hw) {
        intake = hw.get(DcMotorEx.class, INTAKE_MOTOR);
        transfer = hw.get(DcMotorEx.class, TRANSFER_MOTOR);
        gate = hw.get(Servo.class, GATE_SERVO);
        intake.setDirection(INTAKE_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(TRANSFER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[]{intake, transfer}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intakeOut = new HwCache.Power(intake);
        transferOut = new HwCache.Power(transfer);
        gateOut = new HwCache.Position(gate);
        gateOut.set(GATE_CLOSED_POS);
    }

    public void setMode(Mode m) {
        mode = m;
    }

    /**
     * Current reads are slow (~1 ms each, not part of the bulk read). The robot calls this every
     * few loops and it reads ONE motor per call, alternating, so no single loop pays for both.
     */
    public void readOneCurrent() {
        if (readTransferNext) transferAmps = transfer.getCurrent(CurrentUnit.AMPS);
        else intakeAmps = intake.getCurrent(CurrentUnit.AMPS);
        readTransferNext = !readTransferNext;
    }

    /** Forget stale currents when the motors stop, so a jam reading doesn't linger. */
    public void clearCurrents() {
        intakeAmps = 0;
        transferAmps = 0;
    }

    public void write(double nowS) {
        double in, tr;
        boolean open;
        switch (mode) {
            case INTAKE: in = INTAKE_POWER; tr = TRANSFER_STAGE_POWER; open = false; break;
            case FEED:   in = INTAKE_POWER; tr = TRANSFER_FEED_POWER; open = true; break;
            case EJECT:  in = EJECT_POWER;  tr = EJECT_POWER; open = false; break;
            case FEED_HOLD: in = 0; tr = 0; open = true; break;
            default:     in = 0; tr = 0; open = false; break;
        }

        boolean forward = mode == Mode.INTAKE || mode == Mode.FEED;
        if (JAM_DETECT_ENABLED && forward) {
            boolean over = intakeAmps > JAM_CURRENT_AMPS || transferAmps > JAM_CURRENT_AMPS;
            if (over && !unjamming) {
                if (overCurrentSince < 0) overCurrentSince = nowS;
                if (nowS - overCurrentSince > JAM_TIME_S) {
                    unjamUntil = nowS + UNJAM_TIME_S;
                    jams++;
                    overCurrentSince = -1;
                }
            } else if (!over) {
                overCurrentSince = -1;
            }
        } else {
            overCurrentSince = -1;
        }
        unjamming = nowS < unjamUntil;
        if (unjamming) {
            in = EJECT_POWER;
            tr = EJECT_POWER;
            open = false;
        }

        if (open && !gateOpen) gateOpenedAt = nowS;
        gateOpen = open;
        gateOut.set(!Double.isNaN(gateOverride) ? gateOverride : open ? GATE_OPEN_POS : GATE_CLOSED_POS);
        intakeOut.set(in);
        transferOut.set(tr);
    }

    public void stop() {
        mode = Mode.IDLE;
        intakeOut.set(0);
        transferOut.set(0);
        gateOut.set(GATE_CLOSED_POS);
        gateOpen = false;
    }
}
