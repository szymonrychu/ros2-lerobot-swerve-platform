"""aiohttp app for topic scraper API."""

from typing import Any

from aiohttp import web

from .paths import endpoint_to_topic


def create_app(scraper: Any, observer: Any = None) -> web.Application:
    """Create web app bound to a TopicScraper and optional RulesObserver.

    Args:
        scraper: Object exposing get_topics_summary() and get_topic_payload(topic).
        observer: Optional RulesObserver exposing get_rules_summary(), get_rule_result(name).
    """

    app = web.Application()

    async def get_topics(_request: web.Request) -> web.Response:
        return web.json_response({"topics": scraper.get_topics_summary()})

    async def get_topic_by_path(request: web.Request) -> web.Response:
        suffix = request.match_info.get("suffix", "")
        endpoint = f"/topics/{suffix}" if suffix else "/topics"
        topic = endpoint_to_topic(endpoint)
        if not topic:
            return web.json_response({"error": "invalid topic endpoint"}, status=400)
        payload = scraper.get_topic_payload(topic)
        if payload is None:
            return web.json_response({"error": "topic has no sample yet", "topic": topic}, status=404)
        return web.json_response(payload)

    async def get_rules(_request: web.Request) -> web.Response:
        if observer is None:
            return web.json_response({"rules": []})
        return web.json_response({"rules": observer.get_rules_summary()})

    async def get_rule_by_name(request: web.Request) -> web.Response:
        if observer is None:
            return web.json_response({"error": "no observation rules configured"}, status=404)
        name = request.match_info.get("name", "").strip()
        if not name:
            return web.json_response({"error": "rule name required"}, status=400)
        result = observer.get_rule_result(name)
        if result is None:
            return web.json_response({"error": "unknown rule", "name": name}, status=404)
        payload = {
            "name": result.rule_name,
            "comparison": result.comparison,
            "oscillation_detected": result.oscillation_detected,
            "oscillation_score": result.oscillation_score,
            "last_updated_ns": result.last_updated_ns,
        }
        return web.json_response(payload)

    app.router.add_get("/topics", get_topics)
    app.router.add_get("/topics/{suffix:.*}", get_topic_by_path)
    app.router.add_get("/rules", get_rules)
    app.router.add_get("/rules/{name}", get_rule_by_name)
    return app
