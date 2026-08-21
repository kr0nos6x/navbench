from __future__ import annotations

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from navbench.__main__ import _open_loop
from navbench.runlog import CommandMode, RunReplay
from navbench.scenario import load_scenario


ROOT = Path(__file__).resolve().parents[2]


class OpenLoopCliTests(unittest.TestCase):
    def setUp(self) -> None:
        self.scenario_path = ROOT / "scenarios" / "straight.yaml"
        self.scenario = load_scenario(self.scenario_path)

    @staticmethod
    def _fake_plot(_samples: object, path: Path) -> None:
        path.write_bytes(b"test plot")

    def test_open_loop_creates_complete_replayable_run_without_overwrite(self) -> None:
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            root = Path(temporary)
            arguments = SimpleNamespace(
                scenario=self.scenario_path,
                output_root=root,
                run_name="open-loop-test",
            )
            output = io.StringIO()
            with patch("navbench.__main__.save_plot", side_effect=self._fake_plot):
                with contextlib.redirect_stdout(output):
                    _open_loop(arguments)

            result = json.loads(output.getvalue())
            path = root / "open-loop-test"
            self.assertEqual(result["status"], "complete")
            self.assertEqual(Path(result["run"]), path)
            self.assertTrue((path / "trajectory.png").is_file())

            replay = RunReplay(path)
            expected_samples = self.scenario.step_count + 1
            self.assertEqual(len(list(replay.ground_truth())), expected_samples)
            commands = list(replay.commands())
            self.assertEqual(len(commands), expected_samples)
            self.assertTrue(
                all(command.mode is CommandMode.OPEN_LOOP for command in commands)
            )
            self.assertTrue(
                all(command.target_speed_mps == 0.0 for command in commands)
            )
            self.assertEqual(len(list(replay.events())), 2)
            self.assertEqual(len(list(replay.timing())), 1)
            self.assertEqual(len(list(replay.measurements())), 0)
            self.assertEqual(len(list(replay.estimates())), 0)

            with patch("navbench.__main__.save_plot", side_effect=self._fake_plot):
                with self.assertRaises(FileExistsError):
                    _open_loop(arguments)

    def test_plot_failure_preserves_recoverable_incomplete_run(self) -> None:
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            root = Path(temporary)
            arguments = SimpleNamespace(
                scenario=self.scenario_path,
                output_root=root,
                run_name="interrupted",
            )
            with patch(
                "navbench.__main__.save_plot", side_effect=RuntimeError("plot failed")
            ):
                with self.assertRaisesRegex(RuntimeError, "plot failed"):
                    _open_loop(arguments)

            path = root / "interrupted"
            self.assertTrue((path / "INCOMPLETE").is_file())
            replay = RunReplay(path, allow_incomplete=True)
            self.assertFalse(replay.is_complete)
            self.assertEqual(
                len(list(replay.ground_truth())), self.scenario.step_count + 1
            )
            self.assertEqual(replay.summary()["status"], "incomplete")


if __name__ == "__main__":
    unittest.main()
