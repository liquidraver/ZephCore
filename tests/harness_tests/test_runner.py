import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import run


class RunnerTests(unittest.TestCase):
    CASE = {"id": "ONE", "name": "Example", "source": "unit/example.cpp"}

    def test_empty_collection(self):
        with self.assertRaises(ValueError):
            run.select_cases([], None)

    def test_unknown_case(self):
        with self.assertRaises(ValueError):
            run.select_cases([self.CASE], "MISSING")

    def test_catalog_duplicate_ids(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "catalog.json"
            path.write_text(json.dumps({"schema_version": 1, "cases": [self.CASE, self.CASE]}))
            with self.assertRaises(ValueError):
                run.load_catalog(path)

    def test_executable_catalog_agreement(self):
        run.validate_listing("ONE\tExample\n", [self.CASE])
        for output in ("", "ONE\tExample\nONE\tExample\n", "TWO\tExample\n", "ONE\tWrong\n", "garbage"):
            with self.assertRaises(ValueError):
                run.validate_listing(output, [self.CASE])

    def test_child_failure_retains_output(self):
        result = run.command([sys.executable, "-c", "import sys; print('evidence'); sys.exit(7)"])
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["returncode"], 7)
        self.assertIn("evidence", result["stdout"])

    def test_child_timeout(self):
        result = run.command([sys.executable, "-c", "import time; print('started', flush=True); time.sleep(30)"], timeout=1)
        self.assertEqual(result["status"], "ERROR")
        self.assertIn("Timeout", result["stderr"])

    def test_missing_executable(self):
        with tempfile.TemporaryDirectory() as directory:
            result = run.command([str(Path(directory) / "missing")])
        self.assertEqual(result["status"], "ERROR")

    def test_success_requires_matching_marker(self):
        for output in ("", "PASS OLD", "PASS ONE\nextra"):
            with patch.object(run, "command", return_value={"status": "PASS", "stdout": output, "stderr": ""}):
                self.assertEqual(run.execute_case("unused", self.CASE)["status"], "ERROR")

    def test_failure_not_overridden_by_success_text(self):
        with patch.object(run, "command", return_value={"status": "FAIL", "stdout": "PASS ONE", "stderr": "crash"}):
            self.assertEqual(run.execute_case("unused", self.CASE)["status"], "FAIL")

    def test_report_counts_and_xml_escaping(self):
        results = [{"id": str(i), "name": "<&>", "status": status, "stderr": "a < b & c"}
                   for i, status in enumerate(("PASS", "FAIL", "ERROR", "BLOCKED"))]
        report = {"results": results}
        with tempfile.TemporaryDirectory() as directory:
            run.write_reports(Path(directory), report)
            xml = ET.parse(Path(directory) / "junit.xml").getroot()
            self.assertEqual(xml.attrib["tests"], "4")
            self.assertEqual(xml.attrib["failures"], "1")
            self.assertEqual(xml.attrib["errors"], "2")
            self.assertEqual(len(xml.findall("testcase")), 4)
            self.assertEqual(json.loads((Path(directory) / "results.json").read_text())["counts"],
                             {"PASS": 1, "FAIL": 1, "ERROR": 1, "BLOCKED": 1})


if __name__ == "__main__":
    unittest.main()
