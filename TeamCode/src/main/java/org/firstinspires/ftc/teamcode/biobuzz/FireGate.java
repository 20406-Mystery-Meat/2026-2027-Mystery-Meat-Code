package org.firstinspires.ftc.teamcode.biobuzz;

import static org.firstinspires.ftc.teamcode.biobuzz.BuzzConfig.*;

/**
 * Decides, every loop, whether the gate is open and whether the transfer is pushing.
 * Pure logic, no hardware -- ShotController uses it on the robot and tools/sim runs the
 * same class against the flywheel + transfer model.
 *
 * Why it works this way (from tools/sim, flywheel section): the 1620 rpm / 48 mm transfer
 * moves POLLEN ~2 m/s, so queued pieces reach the flywheel only ~35 ms apart, while the
 * gate servo needs ~80 ms to move. The gate is far too slow to let pieces through one at a
 * time, and pieces that hit a wheel still slowed by the previous shot land way short.
 *
 * So the TRANSFER MOTOR meters the pieces, and the gate just opens for the whole burst:
 *  1. FIRE held + everything ready -> the gate opens (and stays open until FIRE is released).
 *  2. After GATE_SERVO_S (the gate is physically open), each time everything is ready the
 *     transfer runs ONE pulse of FEED_PULSE_S -- about one piece of travel -- then stops.
 *     The roller squeeze holds the remaining pieces still while it's stopped.
 *  3. A pulse also ends the instant the flywheel dip shows a piece launched.
 *  4. Then wait until the wheel is back at speed (judged on the filtered rpm) -> next pulse.
 */
public class FireGate {
    /** Command for the gate servo. */
    public boolean gateOpen;
    /** Command for the transfer (feed one piece). */
    public boolean transferOn;
    /** Everything ready (debounced) and not recovering. */
    public boolean ready;
    /** From the end of a pulse (or a detected shot) until the wheel has recovered. */
    public boolean recovering;

    /** Consecutive feed pulses that launched nothing (magazine probably empty). */
    public int dryPulses;
    private boolean pulseLaunched;
    private double readySince = -1, gateOpenedAt = -1, pulseStart = -1, recoverFrom = -1;

    /** @return true while the transfer should push (i.e. a piece is being fed). */
    public boolean update(double nowS, boolean fireHeld, boolean allOk, boolean flywheelOk, boolean shotDetected) {
        // ---- end the current pulse on a launch or when it times out
        if (shotDetected) {
            pulseLaunched = true;
            dryPulses = 0;
        }
        if (transferOn && (shotDetected || nowS - pulseStart >= FEED_PULSE_S)) {
            transferOn = false;
            recovering = true;
            recoverFrom = nowS;
        }
        if (shotDetected && !recovering) {
            recovering = true;
            recoverFrom = nowS;
        }
        // The filtered rpm lags ~half its window: don't trust "ok" until it could have seen a dip.
        if (recovering && !shotDetected && flywheelOk && nowS - recoverFrom >= FLY_FILTER_MS / 1000.0) {
            recovering = false;
            if (!pulseLaunched) dryPulses++;
        }

        boolean ok = allOk && !recovering;
        if (ok) {
            if (readySince < 0) readySince = nowS;
        } else {
            readySince = -1;
        }
        ready = ok && nowS - readySince >= FIRE_READY_DEBOUNCE_S;

        // ---- gate: open for the whole burst
        if (!fireHeld) {
            gateOpen = false;
            transferOn = false;
            gateOpenedAt = -1;
            return false;
        }
        if (!gateOpen && ready) {
            gateOpen = true;
            gateOpenedAt = nowS;
        }

        // ---- start a pulse once the gate is physically open and everything is ready
        boolean gateIsOpen = gateOpen && nowS - gateOpenedAt >= GATE_SERVO_S;
        if (!transferOn && gateIsOpen && ready) {
            transferOn = true;
            pulseStart = nowS;
            pulseLaunched = false;
        }
        return transferOn;
    }

    /** allOk is true but we're still inside the debounce window. */
    public boolean debouncing(boolean allOk) {
        return allOk && !recovering && !ready;
    }
}
