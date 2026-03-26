"""Run topic_scraper_api: ROS spin thread + topic refresh thread + observer thread + aiohttp server."""

import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .app import create_app
from .config import load_config_from_env
from .observer import RulesObserver
from .scraper import TopicScraper


def observer_loop(
    observer: RulesObserver,
    scraper: TopicScraper,
    stop_event: threading.Event,
    interval_s: float,
) -> None:
    """Periodically evaluate observation rules from scraper payloads."""
    while not stop_event.is_set():
        observer.tick(scraper.get_topic_payload)
        stop_event.wait(interval_s)


def spin_node(node: Node, stop_event: threading.Event) -> None:
    """Spin ROS node in a background thread.

    Args:
        node: ROS node.
        stop_event: Event used to stop loop.
    """

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    while rclpy.ok() and not stop_event.is_set():
        executor.spin_once(timeout_sec=0.1)


def refresh_topics_loop(
    scraper: TopicScraper,
    stop_event: threading.Event,
    interval_s: float,
) -> None:
    """Periodically refresh discovered topic subscriptions.

    Args:
        scraper: TopicScraper instance.
        stop_event: Event used to stop loop.
        interval_s: Refresh interval in seconds.
    """

    while not stop_event.is_set():
        scraper.sync_topics()
        stop_event.wait(interval_s)


def main() -> int:
    """Load config, initialize ROS node, start scraper API server."""

    config = load_config_from_env()
    if config is None:
        print(
            "topic_scraper_api config not found. Set TOPIC_SCRAPER_API_CONFIG or deploy to "
            "/etc/ros2/topic_scraper_api/config.yaml",
            file=sys.stderr,
        )
        return 1

    rclpy.init()
    node = Node("topic_scraper_api")
    scraper = TopicScraper(node=node, allowed_types=config.allowed_types)
    observer = RulesObserver(config.observation_rules) if config.observation_rules else None
    stop_event = threading.Event()

    spin_thread = threading.Thread(target=spin_node, args=(node, stop_event), daemon=True)
    refresh_thread = threading.Thread(
        target=refresh_topics_loop,
        args=(scraper, stop_event, config.topic_refresh_interval_s),
        daemon=True,
    )
    spin_thread.start()
    refresh_thread.start()

    if observer is not None:
        obs_interval = max(0.1, config.topic_refresh_interval_s)
        observer_thread = threading.Thread(
            target=observer_loop,
            args=(observer, scraper, stop_event, obs_interval),
            daemon=True,
        )
        observer_thread.start()

    app = create_app(scraper, observer)
    try:
        from aiohttp import web

        web.run_app(app, host=config.host, port=config.port)
    finally:
        stop_event.set()
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()
    return 0
