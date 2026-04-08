"""Smoke tests keeping docs aligned with the K-City competition config."""

from __future__ import annotations

import json
from pathlib import Path
import unittest

from alpamayo1_5.competition.runtime.config_competition import load_competition_config


class CompetitionDocsSmokeTest(unittest.TestCase):
    def test_kcity_docs_reference_competition_profile_and_topics(self) -> None:
        config = load_competition_config("configs/competition_morai_kcity_2026.json")
        quickstart = Path("docs/morai_kcity_2026_quickstart.md").read_text(encoding="utf-8")
        live_usage = Path("docs/morai_live_usage.md").read_text(encoding="utf-8")
        erp_runtime = Path("docs/morai_erp_runtime.md").read_text(encoding="utf-8")
        sim_reference = Path("docs/morai_sim_workspace_reference.md").read_text(encoding="utf-8")
        sim_reference_json = json.loads(
            Path("docs/morai_sim_2025_final_udp_profile.json").read_text(encoding="utf-8")
        )

        for document in (quickstart, live_usage):
            self.assertIn(config.competition_profile.map_name, document)
            self.assertIn(config.competition_profile.vehicle_model, document)
            self.assertIn("Ubuntu 20.04", document)
            self.assertIn("ROS1 Noetic", document)
            self.assertIn("/ctrl_cmd", document)
            self.assertIn("longi type 1", document)
            self.assertIn("/Local/heading", document)
            self.assertIn("/Local/utm", document)
            self.assertIn("/ERP/serial_data", document)
            self.assertIn("historical reference", document.lower())
            self.assertIn("active default", document.lower())
            self.assertIn("diagnostics-only", document.lower())

        self.assertIn("historical reference", sim_reference.lower())
        self.assertIn("morai_msgs", sim_reference)
        self.assertEqual(sim_reference_json["multi_ip"], "192.168.0.100")
        self.assertEqual(sim_reference_json["ctrl_cmd_host_port"], 3300)
        self.assertEqual(sim_reference_json["competition_status_host_port"], 3314)
        self.assertIn("erp-oriented", erp_runtime.lower())
        self.assertIn("/control/serial_data", erp_runtime.lower())
        self.assertIn("/erp/serial_data", erp_runtime.lower())
        self.assertIn("erp_200", erp_runtime.lower())
        self.assertIn("active default", erp_runtime.lower())
        self.assertIn("/ctrl_cmd", erp_runtime.lower())
        self.assertIn("optional", erp_runtime.lower())
        self.assertIn("inherits the `3.0 m` baseline".lower(), erp_runtime.lower())


if __name__ == "__main__":
    unittest.main()
