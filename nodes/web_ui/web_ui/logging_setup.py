"""structlog configuration for web_ui.

Call configure_logging() once at startup before any loggers are created.
Log level is controlled by the DEBUG environment variable:
  DEBUG=true  -> logging.DEBUG (verbose)
  (unset)     -> logging.INFO

Output format:
  JSON   when stdout is not a TTY (container / production)
  Colour ConsoleRenderer when stdout is a TTY (local dev)
"""

from __future__ import annotations

import logging
import os
import sys

import structlog


def configure_logging() -> None:
    """Configure structlog with JSON or console rendering based on environment.

    Reads the DEBUG env var to set log level. Should be called once at
    process startup before importing any module that uses structlog.get_logger().
    """
    debug = os.environ.get("DEBUG", "").lower() in ("1", "true", "yes")
    level = logging.DEBUG if debug else logging.INFO

    shared_processors: list[structlog.types.Processor] = [
        structlog.contextvars.merge_contextvars,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
    ]

    if sys.stdout.isatty():
        renderer: structlog.types.Processor = structlog.dev.ConsoleRenderer()
    else:
        renderer = structlog.processors.JSONRenderer()

    structlog.configure(
        processors=[
            *shared_processors,
            structlog.stdlib.ProcessorFormatter.wrap_for_formatter,
        ],
        logger_factory=structlog.stdlib.LoggerFactory(),
        wrapper_class=structlog.stdlib.BoundLogger,
        cache_logger_on_first_use=True,
    )

    formatter = structlog.stdlib.ProcessorFormatter(
        processor=renderer,
        foreign_pre_chain=shared_processors,
    )

    handler = logging.StreamHandler()
    handler.setFormatter(formatter)

    root_logger = logging.getLogger()
    root_logger.addHandler(handler)
    root_logger.setLevel(level)

    # Quiet noisy libraries
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING if not debug else logging.DEBUG)
    logging.getLogger("uvicorn.error").setLevel(logging.WARNING if not debug else logging.DEBUG)
