"""Structured logging configuration with readable console output."""

from __future__ import annotations

import json
import logging
import logging.handlers
from pathlib import Path
from typing import Any, Dict


class JsonLineFormatter(logging.Formatter):
    """Format one log record as one JSON object."""

    _reserved_names = set(logging.makeLogRecord({}).__dict__.keys())

    def format(self, record: logging.LogRecord) -> str:
        payload: Dict[str, Any] = {
            "timestamp": self.formatTime(record, self.datefmt),
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }
        for key, value in record.__dict__.items():
            if key in self._reserved_names or key.startswith("_"):
                continue
            try:
                json.dumps(value)
                payload[key] = value
            except TypeError:
                payload[key] = repr(value)
        if record.exc_info:
            payload["exception"] = self.formatException(record.exc_info)
        return json.dumps(payload, ensure_ascii=False, sort_keys=True)


def configure_logging(config: Any) -> None:
    """Configure root logging from a ``LoggingConfig`` value object.

    Args:
        config: Object exposing level, console, file, max_bytes, backup_count,
            and json_file attributes.

    Side effects:
        Replaces handlers on the process root logger.
    """
    root_logger = logging.getLogger()
    root_logger.handlers.clear()
    root_logger.setLevel(getattr(logging, str(config.level).upper(), logging.INFO))

    if config.console:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(
            logging.Formatter("%(asctime)s | %(levelname)-8s | %(name)s | %(message)s")
        )
        root_logger.addHandler(console_handler)

    if config.file:
        log_path = Path(config.file).expanduser()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.handlers.RotatingFileHandler(
            filename=str(log_path),
            maxBytes=int(config.max_bytes),
            backupCount=int(config.backup_count),
            encoding="utf-8",
        )
        if config.json_file:
            file_handler.setFormatter(JsonLineFormatter())
        else:
            file_handler.setFormatter(
                logging.Formatter("%(asctime)s | %(levelname)-8s | %(name)s | %(message)s")
            )
        root_logger.addHandler(file_handler)
