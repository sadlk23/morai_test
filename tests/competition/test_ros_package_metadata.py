"""Tests for ROS wrapper package metadata and launch guidance."""

from __future__ import annotations

from pathlib import Path
import unittest


class RosPackageMetadataTest(unittest.TestCase):
    def test_package_metadata_declares_morai_msgs_for_direct_actuation(self) -> None:
        package_xml = Path("ros1/alpamayo1_5_ros/package.xml").read_text(encoding="utf-8")
        cmake_lists = Path("ros1/alpamayo1_5_ros/CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("morai_msgs", package_xml)
        self.assertIn("morai_msgs", cmake_lists)
        self.assertIn("CtrlCmd", package_xml)

    def test_launch_files_document_debug_first_and_morai_msgs_contract(self) -> None:
        live_launch = Path("ros1/alpamayo1_5_ros/launch/run_competition_live.launch").read_text(encoding="utf-8")
        kcity_launch = Path("ros1/alpamayo1_5_ros/launch/run_competition_kcity_2026.launch").read_text(
            encoding="utf-8"
        )
        for launch_text in (live_launch, kcity_launch):
            self.assertIn("debug", launch_text.lower())
            self.assertIn("morai_msgs", launch_text)


if __name__ == "__main__":
    unittest.main()
