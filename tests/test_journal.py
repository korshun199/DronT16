"""Проверки записи журнала только внутри ARM-цикла."""

from pathlib import Path
from tempfile import TemporaryDirectory
from contextlib import redirect_stdout
from io import StringIO
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

            self.assertIsNotNone(journal.session_path)
            contents = journal.session_path.read_text(encoding="utf-8")

            self.assertIn("ARM", contents)
            self.assertIn("ARM session started", contents)
        self.assertIn("sensor", contents)
        self.assertNotIn("before arm", contents)
        self.assertNotIn("after disarm", contents)

    def test_console_and_file_channels_are_independent(self) -> None:
        """Консоль и файл получают только назначенные им категории."""
        with TemporaryDirectory() as directory:
            path = Path(directory) / "channels.log"
            output = StringIO()
            with redirect_stdout(output):
                journal = EventJournal(
                    path,
                    "TEST",
                    console_categories={"RPI"},
                    file_categories={"RPI", "FC"},
                )
                journal.begin_session()
                journal.write("PILOT", "pilot hidden")
                journal.write("RPI", "rpi visible")
                journal.write("FC", "fc file only")
                journal.close()

            console = output.getvalue()
            self.assertIsNotNone(journal.session_path)
            contents = journal.session_path.read_text(encoding="utf-8")

        self.assertIn("rpi visible", console)
        self.assertNotIn("pilot hidden", console)
        self.assertNotIn("fc file only", console)
        self.assertIn("rpi visible", contents)
        self.assertIn("fc file only", contents)
        self.assertNotIn("pilot hidden", contents)

    def test_fault_closes_active_session(self) -> None:
        """После сбоя текущая ARM-сессия закрывается и новые строки не пишутся."""
        with TemporaryDirectory() as directory:
            path = Path(directory) / "fault.log"
            journal = EventJournal(path, "TEST")
            journal.begin_session()
            journal.write("RPI", "before fault", console=False)
            journal.write("RPI", "SESSION END: FAULT", console=False)
            journal.end_session()
            journal.write("RPI", "after fault", console=False)
            journal.close()

            self.assertIsNotNone(journal.session_path)
            contents = journal.session_path.read_text(encoding="utf-8")

        self.assertIn("before fault", contents)
        self.assertIn("SESSION END: FAULT", contents)
        self.assertNotIn("after fault", contents)

    def test_each_arm_session_gets_a_unique_timestamped_file(self) -> None:
        """Каждый ARM-цикл сохраняется отдельным неперезаписываемым файлом."""
        with TemporaryDirectory() as directory:
            base = Path(directory) / "simulator_filesafe.log"
            journal = EventJournal(base, "TEST")
            journal.begin_session()
            first = journal.session_path
            journal.write("PILOT", "first arm", console=False)
            journal.end_session()
            journal.begin_session()
            second = journal.session_path
            journal.write("PILOT", "second arm", console=False)
            journal.end_session()
            journal.close()

            self.assertIsNotNone(first)
            self.assertIsNotNone(second)
            self.assertNotEqual(first, second)
            self.assertRegex(first.name, r"simulator_filesafe_\d{8}_\d{6}_\d{3}\.log")
            self.assertIn("first arm", first.read_text(encoding="utf-8"))
            self.assertIn("second arm", second.read_text(encoding="utf-8"))

    def test_async_file_writer_flushes_before_disarm_closes_session(self) -> None:
        """Фоновая запись не теряет события ARM-цикла при немедленном DISARM."""
        with TemporaryDirectory() as directory:
            path = Path(directory) / "async.log"
            journal = EventJournal(path, "TEST", asynchronous_file_write=True)
            journal.begin_session()
            for number in range(100):
                journal.write("FC", f"sensor={number}", console=False)
            journal.end_session()
            journal.close()

            self.assertIsNotNone(journal.session_path)
            contents = journal.session_path.read_text(encoding="utf-8")

        self.assertIn("sensor=0", contents)
        self.assertIn("sensor=99", contents)
        self.assertEqual(journal.dropped_file_events, 0)
