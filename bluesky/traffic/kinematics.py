""" BlueSky kinematics implementation.

This module contains the point-mass kinematics that integrate the aircraft
state (airspeed, heading, vertical speed, ground speed, track and position)
each simulation step. It is factored out of Traffic into a separate,
replaceable Entity (upstream PR #655) so that the kinematics model can be
swapped for an alternative implementation with Kinematics.select().

The fundamental aircraft state (lat, lon, alt, tas, hdg, vs, ...) lives on
Traffic and is shared with the rest of BlueSky; the kinematics model reads and
writes it through the global bs.traf reference, exactly like the other flight
models (autopilot, ASAS, ...). The state produced by the integration itself
(the longitudinal/vertical accelerations, the turn/altitude-capture switches
and the turn-rate limiter memory) is owned here as traffic arrays.

Besides the default (which keeps the CDaRR fork's turn-rate limiter), three
UAS implementations are provided, following OpenCDaRR's kinematics split:
- MultirotorKinematics: every aircraft flies the multirotor model
- FixedWingKinematics: every aircraft flies the fixed-wing model
- UASKinematics: mixed traffic; each aircraft flies the model named by its
  type's UAS_PERFORMANCE entry (M600 -> multirotor, Wingcopter/Trinity ->
  fixed-wing), and unlisted types fly the default point-mass.
Select one *before* creating traffic, e.g. UASKinematics.select().

The horizontal (airspeed + heading) step of each model lives in a module
function that reads the traffic state and returns the new values without
writing them, so the single-model classes and the mixed dispatcher share one
copy of the physics.
"""
import numpy as np

import bluesky as bs
from bluesky.core import Entity
from bluesky.tools.aero import fpm, ft, g0, Rearth, vtas2cas, vtas2mach


# Per-actype UAS parameters read by the UAS kinematics implementations below.
# "model" names the physics UASKinematics dispatches this type to; the speed
# envelope (v_min/v_max) is NOT duplicated here: it lives in the OpenAP rotor
# database (resources/performance/OpenAP/rotor/aircraft.json) and is read back
# through bs.traf.perf. Sources:
#   M600       DJI Matrice 600 (multirotor). ax = 5.0 m/s2 is OpenCDaRR's M600
#              value (deliberate override; the OpenAP rotor model's constant is
#              3.5 m/s2).
#   WINGCOPTER Wingcopter 198, tilt-rotor fixed-wing VTOL. Cruise 100 km/h,
#              top speed 150 km/h (manufacturer specsheet). bank/roll_rate are
#              the typical-small-UAV values used by OpenCDaRR's SMALL_FIXEDWING
#              (operational bank ~44 deg, roll rate 60 deg/s, Reyner & Liem,
#              Drones 2026); ax = 2.0 m/s2 ASSUMED (thrust-limited airspeed
#              changes are slow on a fixed-wing).
#   TRINITY    Quantum Systems Trinity F90+, fixed-wing VTOL. Cruise 17 m/s,
#              top speed ~85 km/h (manufacturer). Same assumed bank/roll/ax as
#              the Wingcopter.
UAS_PERFORMANCE = {
    "M600":       {"model": "multirotor", "ax": 5.0},
    "WINGCOPTER": {"model": "fixedwing", "ax": 2.0, "bank": 44.0, "roll_rate_max": 60.0},
    "TRINITY":    {"model": "fixedwing", "ax": 2.0, "bank": 44.0, "roll_rate_max": 60.0},
}

# Model codes for UASKinematics' per-aircraft dispatch
MODEL_DEFAULT, MODEL_MULTIROTOR, MODEL_FIXEDWING = 0, 1, 2
_MODEL_CODE = {"multirotor": MODEL_MULTIROTOR, "fixedwing": MODEL_FIXEDWING}


def default_horizontal(prev_turnrate):
    """ Default horizontal step (CDaRR flavour) for all aircraft: airspeed ramp
        at perf.axmax; heading via the bank-angle model, or the fork's
        rate-limited turn model where max_tr/max_dtr2 are finite (e.g. M600).
        Returns (tas, hdg, ax, swhdgsel, prev_turnrate) without writing them.
    """
    traf = bs.traf
    dt = bs.sim.simdt

    # Compute horizontal acceleration
    delta_spd = traf.aporasas.tas - traf.tas
    need_ax = np.abs(delta_spd) > np.abs(dt * traf.perf.axmax)
    ax = need_ax * np.sign(delta_spd) * traf.perf.axmax
    newtas = np.where(need_ax, traf.tas + ax * dt, traf.aporasas.tas)

    hdg_err = (traf.aporasas.hdg - traf.hdg + 180.0) % 360.0 - 180.0  # [deg], signed

    # Rate-limited turn rate for aircraft with a turn rate limiter (e.g. M600)
    trlim_mask      = np.isfinite(traf.max_tr)
    prev_tr         = prev_turnrate
    raw_tr          = np.clip(hdg_err, -traf.max_tr, traf.max_tr)    # desired turn rate [deg/s]
    tr_step_needed  = raw_tr - prev_tr                                # [deg/s]
    tr_step_max     = traf.max_dtr2 * dt                              # [deg/s] allowed change this step
    tr_step_limited = np.clip(tr_step_needed, -tr_step_max, tr_step_max)
    limited_tr      = np.clip(prev_tr + tr_step_limited, -traf.max_tr, traf.max_tr)  # [deg/s]

    # Bank-angle turn rate for unconstrained aircraft (tan phi = omega*V/g), with sign
    default_tr = np.sign(hdg_err) * np.degrees(
        g0 * np.tan(np.where(traf.ap.turnphi > traf.eps * traf.eps,
                             traf.ap.turnphi, traf.ap.bankdef))
        / np.maximum(newtas, traf.eps)
    )

    turnrate = np.where(trlim_mask, limited_tr, default_tr)  # [deg/s], signed
    newprev = np.where(trlim_mask, turnrate, 0.0)

    swhdgsel = np.abs(hdg_err) > np.abs(dt * turnrate)
    newhdg = np.where(swhdgsel, (traf.hdg + dt * turnrate), traf.aporasas.hdg) % 360.0

    return newtas, newhdg, ax, swhdgsel, newprev


def multirotor_horizontal(axlim):
    """ Multirotor horizontal step, after OpenCDaRR's Multirotor model: the
        velocity vector moves directly toward the commanded vector under the
        isotropic acceleration limit axlim; no coupled heading or turn-rate
        limit. Returns (tas, hdg, ax, swhdgsel) without writing them.
    """
    traf = bs.traf
    dt = bs.sim.simdt

    # Current and commanded horizontal velocity vectors
    cure = traf.tas * np.sin(np.radians(traf.hdg))
    curn = traf.tas * np.cos(np.radians(traf.hdg))
    tgte = traf.aporasas.tas * np.sin(np.radians(traf.aporasas.hdg))
    tgtn = traf.aporasas.tas * np.cos(np.radians(traf.aporasas.hdg))

    # Move the velocity vector at most axlim*dt toward the commanded one
    # (snap onto it when it is reachable within this step)
    dist = np.hypot(tgte - cure, tgtn - curn)
    scale = np.minimum(1.0, axlim * dt / np.maximum(dist, traf.eps))
    newe = cure + scale * (tgte - cure)
    newn = curn + scale * (tgtn - curn)

    newtas = np.hypot(newe, newn)
    ax = (newtas - traf.tas) / dt
    newhdg = np.where(newtas > traf.eps,
                      np.degrees(np.arctan2(newe, newn)), traf.hdg) % 360.0

    # Still converging toward the commanded velocity vector?
    swhdgsel = dist > axlim * dt

    return newtas, newhdg, ax, swhdgsel


def fixedwing_horizontal(axlim, bank, roll_rate_max):
    """ Fixed-wing horizontal step, after OpenCDaRR's FixedWing model:
        airspeed ramp at axlim; coordinated-turn heading via a bank limited
        structurally (BANK cmd / autopilot turn bank) and by stall-in-turn
        (cos phi >= (v_stall/V)^2 with v_stall = positive perf.vmin), reached
        at a finite roll rate from the current bank. Returns
        (tas, hdg, ax, swhdgsel, bank, turnrate) without writing them.
    """
    traf = bs.traf
    dt = bs.sim.simdt

    # Airspeed ramp (same law as the default, with the UAS accel override)
    delta_spd = traf.aporasas.tas - traf.tas
    need_ax = np.abs(delta_spd) > np.abs(dt * axlim)
    ax = need_ax * np.sign(delta_spd) * axlim
    newtas = np.where(need_ax, traf.tas + ax * dt, traf.aporasas.tas)

    v = np.maximum(newtas, traf.eps)

    # Bank authority: the structural limit (BANK cmd / autopilot turn bank),
    # tightened by the stall-in-turn constraint at the current airspeed
    phi_struct = np.degrees(np.where(traf.ap.turnphi > traf.eps * traf.eps,
                                     traf.ap.turnphi, traf.ap.bankdef))
    v_stall = np.maximum(traf.perf.vmin, 0.0)  # non-positive vmin = no stall floor
    phi_stall = np.degrees(np.arccos(np.minimum(1.0, (v_stall / v) ** 2)))
    phi_max = np.minimum(phi_struct, phi_stall)

    # Desired turn rate (proportional in the heading error) within the
    # speed-dependent cap omega_max = g*tan(phi_max)/V, and the bank for it
    hdg_err = (traf.aporasas.hdg - traf.hdg + 180.0) % 360.0 - 180.0  # [deg], signed
    w_max = np.degrees(g0 * np.tan(np.radians(phi_max)) / v)
    w_des = np.clip(hdg_err, -w_max, w_max)
    phi_des = np.degrees(np.arctan(np.radians(w_des) * v / g0))

    # Finite roll: the bank moves toward the desired bank at most
    # roll_rate_max*dt per step, then the achieved bank turns the aircraft
    roll_step = np.clip(phi_des - bank, -roll_rate_max * dt, roll_rate_max * dt)
    newbank = np.clip(bank + roll_step, -phi_max, phi_max)

    turnrate = np.degrees(g0 * np.tan(np.radians(newbank)) / v)  # [deg/s], signed
    swhdgsel = np.abs(hdg_err) > np.abs(dt * turnrate)
    newhdg = np.where(swhdgsel, traf.hdg + dt * turnrate, traf.aporasas.hdg) % 360.0

    return newtas, newhdg, ax, swhdgsel, newbank, turnrate


class Kinematics(Entity, replaceable=True):
    """ Default BlueSky point-mass kinematics (CDaRR flavour).

    Integrates airspeed/heading/vertical speed, derives ground speed and
    track (including wind), and updates the aircraft position each step.
    Heading is turned either with the bank-angle model (tan phi), or, for
    aircraft with a finite max_tr/max_dtr2 (e.g. the M600), with the fork's
    rate-limited turn model.
    """

    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            # Accelerations produced by the integration
            self.ax       = np.array([])              # [m/s2] current longitudinal acceleration
            self.az       = np.array([])              # [m/s2] current vertical acceleration

            # Guidance-mode switches produced by the integration
            self.swhdgsel = np.array([], dtype=bool)  # whether aircraft is turning
            self.swaltsel = np.array([], dtype=bool)  # whether altitude capture/hold is engaged

            # Turn-rate limiter memory (used with Traffic's max_tr/max_dtr2)
            self.prev_turnrate = np.array([])         # [deg/s] turn rate at previous timestep

    def create(self, n=1):
        super().create(n)

        self.ax[-n:]            = 0.0
        self.az[-n:]            = 0.0
        self.swhdgsel[-n:]      = False
        self.swaltsel[-n:]      = False
        self.prev_turnrate[-n:] = 0.0

    def update(self):
        """ Perform one kinematic integration step for all aircraft. """
        self.update_airspeed()
        self.update_groundspeed()
        self.update_pos()

    def update_airspeed(self):
        traf = bs.traf
        traf.tas, traf.hdg, self.ax, self.swhdgsel, self.prev_turnrate = \
            default_horizontal(self.prev_turnrate)
        traf.cas = vtas2cas(traf.tas, traf.alt)
        traf.M = vtas2mach(traf.tas, traf.alt)

        self.update_vertical_speed()

    def update_vertical_speed(self):
        """ Update vertical speed (alt select, capture and hold autopilot mode). """
        traf = bs.traf
        delta_alt = traf.aporasas.alt - traf.alt
        # Old dead band version:
        #        self.swaltsel = np.abs(delta_alt) > np.maximum(
        #            10 * ft, np.abs(2 * bs.sim.simdt * traf.vs))

        # Update version: time based engage of altitude capture (to adapt for UAV vs airliner scale)
        self.swaltsel = np.abs(delta_alt) > 1.05 * np.maximum(np.abs(bs.sim.simdt * traf.aporasas.vs),
                                                              np.abs(bs.sim.simdt * traf.vs))
        target_vs = self.swaltsel * np.sign(delta_alt) * np.abs(traf.aporasas.vs)
        delta_vs = target_vs - traf.vs
        # print(delta_vs / fpm)
        need_az = np.abs(delta_vs) > 300 * fpm   # small threshold
        self.az = need_az * np.sign(delta_vs) * (300 * fpm)   # fixed vertical acc approx 1.6 m/s^2
        traf.vs = np.where(need_az, traf.vs + self.az * bs.sim.simdt, target_vs)
        traf.vs = np.where(np.isfinite(traf.vs), traf.vs, 0)    # fix vs nan issue

    def update_groundspeed(self):
        traf = bs.traf
        # Compute ground speed and track from heading, airspeed and wind
        if traf.wind.winddim == 0:  # no wind
            traf.gsnorth  = traf.tas * np.cos(np.radians(traf.hdg))
            traf.gseast   = traf.tas * np.sin(np.radians(traf.hdg))

            traf.gs  = traf.tas
            traf.trk = traf.hdg
            traf.windnorth[:], traf.windeast[:] = 0.0, 0.0

        else:
            applywind = traf.alt > 50. * ft  # Only apply wind when airborne

            vnwnd, vewnd = traf.wind.getdata(traf.lat, traf.lon, traf.alt)
            traf.windnorth[:], traf.windeast[:] = vnwnd, vewnd
            traf.gsnorth  = traf.tas * np.cos(np.radians(traf.hdg)) + traf.windnorth * applywind
            traf.gseast   = traf.tas * np.sin(np.radians(traf.hdg)) + traf.windeast * applywind

            traf.gs  = np.logical_not(applywind) * traf.tas + \
                       applywind * np.sqrt(traf.gsnorth**2 + traf.gseast**2)

            traf.trk = np.logical_not(applywind) * traf.hdg + \
                       applywind * np.degrees(np.arctan2(traf.gseast, traf.gsnorth)) % 360.

        traf.work += (traf.perf.thrust * bs.sim.simdt * np.sqrt(traf.gs * traf.gs + traf.vs * traf.vs))

    def update_pos(self):
        traf = bs.traf
        # Update position
        traf.alt = np.where(self.swaltsel, np.round(traf.alt + traf.vs * bs.sim.simdt, 6), traf.aporasas.alt)
        traf.lat = traf.lat + np.degrees(bs.sim.simdt * traf.gsnorth / Rearth)
        traf.coslat = np.cos(np.deg2rad(traf.lat))
        traf.lon = traf.lon + np.degrees(bs.sim.simdt * traf.gseast / traf.coslat / Rearth)
        traf.distflown += traf.gs * bs.sim.simdt


class MultirotorKinematics(Kinematics):
    """ Multirotor point-mass kinematics, after OpenCDaRR's Multirotor model.

    Every aircraft flies the multirotor model: the horizontal (air-)velocity
    vector moves directly toward the commanded vector under an isotropic
    acceleration limit — no coupled heading, no turn-rate limit. A direction
    change is a bounded step in velocity space, and the vehicle can slow down
    and stop. Heading equals the direction of motion (BlueSky carries no
    decoupled yaw channel), so Traffic's max_tr/max_dtr2 limiter is not used
    by this model.

    The acceleration limit is UAS_PERFORMANCE["<actype>"]["ax"] when the
    aircraft type has an entry (e.g. the M600), else the performance model's
    axmax. Select with MultirotorKinematics.select() before creating traffic.
    """

    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            self.axiso = np.array([])  # [m/s2] isotropic acceleration limit (nan = perf.axmax)

    def create(self, n=1):
        super().create(n)

        for i in range(-n, 0):
            entry = UAS_PERFORMANCE.get(bs.traf.type[i].upper(), {})
            self.axiso[i] = entry.get("ax", np.nan)

    def update_airspeed(self):
        traf = bs.traf
        axlim = np.where(np.isnan(self.axiso), traf.perf.axmax, self.axiso)
        traf.tas, traf.hdg, self.ax, self.swhdgsel = multirotor_horizontal(axlim)
        traf.cas = vtas2cas(traf.tas, traf.alt)
        traf.M = vtas2mach(traf.tas, traf.alt)

        self.update_vertical_speed()


class FixedWingKinematics(Kinematics):
    """ Fixed-wing coordinated-turn kinematics, after OpenCDaRR's FixedWing model.

    Every aircraft flies the fixed-wing model. Non-holonomic: the aircraft
    turns by banking (yaw rate g*tan(phi)/V), the bank limit is tightened by
    the stall-in-turn constraint (cos phi >= (v_stall/V)^2), and the bank
    angle changes at a finite roll rate, which makes it part of the state.
    The stall speed is the performance model's vmin (only where positive), so
    for the UAS types it comes from the OpenAP rotor database. The structural
    bank limit is the regular BlueSky one (ap.turnphi/ap.bankdef, i.e. the
    BANK command), seeded from UAS_PERFORMANCE for known types. Select with
    FixedWingKinematics.select() before creating traffic.
    """

    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            self.bank          = np.array([])  # [deg] current roll angle
            self.roll_rate_max = np.array([])  # [deg/s] max roll rate
            self.axcmd         = np.array([])  # [m/s2] airspeed accel limit (nan = perf.axmax)

    def create(self, n=1):
        super().create(n)

        self.bank[-n:] = 0.0
        for i in range(-n, 0):
            entry = UAS_PERFORMANCE.get(bs.traf.type[i].upper(), {})
            self.roll_rate_max[i] = entry.get("roll_rate_max", 60.0)
            self.axcmd[i] = entry.get("ax", np.nan)
            if "bank" in entry:
                bs.traf.ap.bankdef[i] = np.radians(entry["bank"])

    def update_airspeed(self):
        traf = bs.traf
        axlim = np.where(np.isnan(self.axcmd), traf.perf.axmax, self.axcmd)
        traf.tas, traf.hdg, self.ax, self.swhdgsel, self.bank, self.prev_turnrate = \
            fixedwing_horizontal(axlim, self.bank, self.roll_rate_max)
        traf.cas = vtas2cas(traf.tas, traf.alt)
        traf.M = vtas2mach(traf.tas, traf.alt)

        self.update_vertical_speed()


class UASKinematics(Kinematics):
    """ Mixed-traffic UAS kinematics: the model is auto-detected per aircraft.

    Each aircraft flies the model named by its type's UAS_PERFORMANCE entry —
    "multirotor" (M600) or "fixedwing" (Wingcopter, Trinity) — and any type
    without an entry flies the default point-mass kinematics, so multirotors,
    fixed-wings and regular aircraft can share one simulation. Adding a type
    is one UAS_PERFORMANCE row. Select with UASKinematics.select() before
    creating traffic.

    Each step computes the three candidate integrations (they share the
    module-level horizontal-step functions with the single-model classes) and
    keeps each aircraft's own lane.
    """

    def __init__(self):
        super().__init__()

        with self.settrafarrays():
            self.model         = np.array([], dtype=np.int8)  # MODEL_* code per aircraft
            self.ax_uas        = np.array([])  # [m/s2] accel limit from the table (nan = perf.axmax)
            self.bank          = np.array([])  # [deg] current roll angle (fixed-wing lanes)
            self.roll_rate_max = np.array([])  # [deg/s] max roll rate (fixed-wing lanes)

    def create(self, n=1):
        super().create(n)

        self.bank[-n:] = 0.0
        for i in range(-n, 0):
            entry = UAS_PERFORMANCE.get(bs.traf.type[i].upper(), {})
            self.model[i] = _MODEL_CODE.get(entry.get("model"), MODEL_DEFAULT)
            self.ax_uas[i] = entry.get("ax", np.nan)
            self.roll_rate_max[i] = entry.get("roll_rate_max", 60.0)
            if "bank" in entry:
                bs.traf.ap.bankdef[i] = np.radians(entry["bank"])

    def update_airspeed(self):
        traf = bs.traf
        is_mr = self.model == MODEL_MULTIROTOR
        is_fw = self.model == MODEL_FIXEDWING
        axlim = np.where(np.isnan(self.ax_uas), traf.perf.axmax, self.ax_uas)

        d_tas, d_hdg, d_ax, d_sw, d_prev = default_horizontal(self.prev_turnrate)
        m_tas, m_hdg, m_ax, m_sw = multirotor_horizontal(axlim)
        f_tas, f_hdg, f_ax, f_sw, f_bank, f_tr = \
            fixedwing_horizontal(axlim, self.bank, self.roll_rate_max)

        traf.tas      = np.select([is_mr, is_fw], [m_tas, f_tas], d_tas)
        traf.hdg      = np.select([is_mr, is_fw], [m_hdg, f_hdg], d_hdg)
        self.ax       = np.select([is_mr, is_fw], [m_ax, f_ax], d_ax)
        self.swhdgsel = np.select([is_mr, is_fw], [m_sw, f_sw], d_sw)
        self.prev_turnrate = np.select([is_mr, is_fw], [np.zeros(traf.ntraf), f_tr], d_prev)
        self.bank     = np.where(is_fw, f_bank, 0.0)
        traf.cas = vtas2cas(traf.tas, traf.alt)
        traf.M = vtas2mach(traf.tas, traf.alt)

        self.update_vertical_speed()
