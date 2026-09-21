"""Проверки записи журнала только внутри ARM-цикла."""

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from src.diagnostics.journal import EventJournal


class EventJournalTest(unittest.TestCase):
    """Проверяет границы ARM -> DISARM для файлового журнала."""

    def test_file_contains_only_active_session(self) -> None:
        """До ARM и после DISARM события остаются только в терминале."""
        with TemporaryDirectory() as directory:
            path = Path(directory) / "test.log"
            journal = EventJournal(path, "TEST")
            journal.write("RPI", "before arm", console=False)
            journal.begin_session()
            journal.write("PILOT", "ARM", console=False)
            journal.write("FC", "sensor", console=False)
            journal.end_session()
            journal.write("RPI", "after disarm", console=False)
            journal.close()

            contents = path.read_text(encoding="utf-8")

        self.assertIn("ARM", contents)
        self.assertIn("sensor", contents)
        self.assertNotIn("before arm", contents)
        self.assertNotIn("after disarm", contents)
