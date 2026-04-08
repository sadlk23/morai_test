"""Smoke tests keeping docs aligned with the K-City competition config."""

from __future__ import annotations

from pathlib import Path
import unittest

from alpamayo1_5.competition.runtime.config_competition import load_competition_config


class CompetitionDocsSmokeTest(unittest.TestCase):
    def test_kcity_docs_reference_competition_profile_and_topics(self) -> None:
        config = load_competition_config("configs/competition_morai_kcity_2026.json")
        quickstart = Path("docs/morai_kcity_2026_quickstart.md").read_text(encoding="utf-8")
        live_usage = Path("docs/morai_live_usage.md").read_text(encoding="utf-8")

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


if __name__ == "__main__":
    unittest.main()
