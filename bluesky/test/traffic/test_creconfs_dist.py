"""Tests for Traffic.creconfs_dist — verifies that the created aircraft
is placed at the requested initial separation distance from the target."""
import pytest
import numpy as np
import bluesky as bs
from bluesky.tools import geo
from bluesky.tools.aero import nm


# ---------------------------------------------------------------------------
# Fixture
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def traf():
    bs.settings.is_sim = True
    bs.init()
    yield bs.traf


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def actual_dist_nm(traf, idx_target, idx_intruder):
    """Return great-circle distance (NM) between two aircraft by index."""
    return geo.kwikdist(
        traf.lat[idx_target], traf.lon[idx_target],
        traf.lat[idx_intruder], traf.lon[idx_intruder],
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestCreconfsDist:
    """Validate that creconfs_dist places the intruder at the requested dist."""

    def _setup_target(self, traf, acid, lat=52.0, lon=4.0, hdg=90.0,
                      alt=10000.0, spd=250.0):
        """Create a target aircraft and return its index."""
        from bluesky.tools.aero import ft, kts
        traf.cre(acid, "B744", lat, lon, hdg, alt * ft, spd * kts)
        return traf.id.index(acid)

    def test_basic_distance(self, traf):
        """Intruder should be placed ~5 NM from target (head-on)."""
        traf.reset()
        target_idx = self._setup_target(traf, "TGT1", hdg=90.0)

        desired_dist_nm = 5.0
        traf.creconfs_dist("INT1", "B744", target_idx,
                           dpsi=180.0, dcpa=0.0, dist=desired_dist_nm)

        intruder_idx = traf.id.index("INT1")
        dist = actual_dist_nm(traf, target_idx, intruder_idx)

        assert abs(dist - desired_dist_nm) < 0.05, (
            f"Expected {desired_dist_nm} NM, got {dist:.4f} NM"
        )

    def test_various_distances(self, traf):
        """Check multiple separation distances with an angled conflict.

        Note: creconfs enforces a minimum initial distance of asas_pzr (5 NM
        by default) when dcpa < pzr, because it always places the intruder at
        least at the edge of the protection zone.  Only request distances that
        are achievable (>= pzr = 5 NM here).
        """
        for desired_nm in [6.0, 10.0, 20.0]:
            traf.reset()
            target_idx = self._setup_target(traf, "TGT2", hdg=0.0)
            traf.creconfs_dist("INT2", "B744", target_idx,
                               dpsi=45.0, dcpa=1.0, dist=desired_nm)
            intruder_idx = traf.id.index("INT2")
            dist = actual_dist_nm(traf, target_idx, intruder_idx)
            assert abs(dist - desired_nm) < 0.1, (
                f"dist={desired_nm} NM: got {dist:.4f} NM"
            )

    def test_dist_larger_than_dcpa(self, traf):
        """dist must be >= dcpa; the geometry should still resolve cleanly."""
        traf.reset()
        target_idx = self._setup_target(traf, "TGT3", hdg=180.0)

        desired_nm = 8.0
        dcpa_nm = 2.0
        traf.creconfs_dist("INT3", "B744", target_idx,
                           dpsi=90.0, dcpa=dcpa_nm, dist=desired_nm)
        intruder_idx = traf.id.index("INT3")
        dist = actual_dist_nm(traf, target_idx, intruder_idx)

        assert abs(dist - desired_nm) < 0.1, (
            f"Expected {desired_nm} NM, got {dist:.4f} NM"
        )

    def test_with_custom_speed(self, traf):
        """Verify dist is correct when intruder speed is specified (spd arg)."""
        traf.reset()
        target_idx = self._setup_target(traf, "TGT4", hdg=270.0)

        desired_nm = 6.0
        traf.creconfs_dist("INT4", "B744", target_idx,
                           dpsi=180.0, dcpa=0.5, dist=desired_nm, spd=200.0)
        intruder_idx = traf.id.index("INT4")
        dist = actual_dist_nm(traf, target_idx, intruder_idx)

        assert abs(dist - desired_nm) < 0.1, (
            f"Expected {desired_nm} NM with custom spd, got {dist:.4f} NM"
        )

    def test_same_track_vrel_zero_fallback(self, traf):
        """When dpsi=0 and same speed, vrel≈0; method should not raise."""
        traf.reset()
        target_idx = self._setup_target(traf, "TGT5", hdg=90.0)

        desired_nm = 5.0
        # dpsi=0 and same gs → vrel ≈ 0, triggers fallback path
        traf.creconfs_dist("INT5", "B744", target_idx,
                           dpsi=0.0, dcpa=0.0, dist=desired_nm)
        # Aircraft should have been created (no crash / exception)
        assert "INT5" in traf.id

    def test_creconfs_dist_equals_creconfs_at_tlosh(self, traf):
        """creconfs_dist should produce the same aircraft position as calling
        creconfs with the tlosh that creconfs_dist internally derives."""
        from math import sqrt, radians, cos, sin
        traf.reset()
        target_idx = self._setup_target(traf, "TGT6", hdg=90.0)

        dpsi = 120.0
        dcpa_nm = 1.5
        dist_nm = 8.0

        # Replicate creconfs_dist's internal tlosh derivation exactly
        gsref = traf.gs[target_idx]
        trkref = radians(traf.trk[target_idx])
        trk = trkref + radians(dpsi)
        gsn = gsref * cos(trk)
        gse = gsref * sin(trk)
        vreln = gsref * cos(trkref) - gsn
        vrele = gsref * sin(trkref) - gse
        vrel = sqrt(vreln**2 + vrele**2)

        dist_m = dist_nm * nm
        cpa_m  = dcpa_nm * nm
        pzr    = bs.settings.asas_pzr * nm
        drelcpa = sqrt(max(dist_m**2 - cpa_m**2, 0.0))
        pzr_correction = 0.0 if cpa_m > pzr else sqrt(pzr**2 - cpa_m**2)
        tlosh_expected = max(drelcpa - pzr_correction, 0.0) / vrel

        # Create via creconfs_dist
        traf.creconfs_dist("INTD", "B744", target_idx,
                           dpsi=dpsi, dcpa=dcpa_nm, dist=dist_nm)
        lat_d = traf.lat[-1]
        lon_d = traf.lon[-1]

        traf.delete(len(traf.lat) - 1)

        # Re-fetch target after delete
        target_idx2 = traf.id.index("TGT6")

        # Create via creconfs with the exactly-derived tlosh — positions match
        traf.creconfs("INTC", "B744", target_idx2,
                      dpsi=dpsi, dcpa=dcpa_nm, tlosh=tlosh_expected)
        lat_c = traf.lat[-1]
        lon_c = traf.lon[-1]

        assert abs(lat_d - lat_c) < 1e-6, f"lat mismatch: {lat_d} vs {lat_c}"
        assert abs(lon_d - lon_c) < 1e-6, f"lon mismatch: {lon_d} vs {lon_c}"
