"""aiohttp app for topic scraper API."""

import asyncio
from typing import Any

from aiohttp import web

from .paths import PREVIEW_PREFIX, STREAM_PREFIX, endpoint_to_topic, preview_endpoint_to_topic, stream_endpoint_to_topic

MJPEG_BOUNDARY = b"frame"
MJPEG_STREAM_INTERVAL_S = 0.05


def create_app(scraper: Any, observer: Any = None) -> web.Application:
    """Create web app bound to a TopicScraper and optional RulesObserver.

    Args:
        scraper: Object exposing get_topics_summary(), get_topic_payload(topic),
            get_image_topics_summary(), get_latest_jpeg(topic).
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

    async def get_streams_list(_request: web.Request) -> web.Response:
        return web.json_response({"streams": scraper.get_image_topics_summary()})

    async def get_stream_by_path(request: web.Request) -> web.Response:
        suffix = request.match_info.get("suffix", "").strip().strip("/")
        path = f"{STREAM_PREFIX}/{suffix}" if suffix else STREAM_PREFIX
        topic = stream_endpoint_to_topic(path)
        if not topic:
            return web.json_response({"error": "invalid stream endpoint", "path": path}, status=400)
        if path.endswith("/mjpg"):
            return await _stream_mjpeg(request, scraper, topic)
        return _stream_html_response(path, topic)

    async def get_previews_list(_request: web.Request) -> web.Response:
        return web.json_response({"previews": scraper.get_image_topics_summary()})

    async def get_preview_by_path(request: web.Request) -> web.Response:
        suffix = request.match_info.get("suffix", "").strip().strip("/")
        path = f"{PREVIEW_PREFIX}/{suffix}" if suffix else PREVIEW_PREFIX
        topic = preview_endpoint_to_topic(path)
        if not topic:
            return web.json_response({"error": "invalid preview endpoint", "path": path}, status=400)
        if path.endswith("/image.jpg"):
            jpeg = scraper.get_latest_jpeg(topic)
            if jpeg is None:
                return web.json_response(
                    {"error": "no image sample yet", "topic": topic},
                    status=404,
                    content_type="application/json",
                )
            return web.Response(body=jpeg, content_type="image/jpeg")
        return _preview_html_response(path, topic)

    app.router.add_get("/topics", get_topics)
    app.router.add_get(r"/topics/{suffix:.*}", get_topic_by_path)
    app.router.add_get("/rules", get_rules)
    app.router.add_get("/rules/{name}", get_rule_by_name)
    app.router.add_get("/streams", get_streams_list)
    app.router.add_get(r"/streams/{suffix:.*}", get_stream_by_path)
    app.router.add_get("/previews", get_previews_list)
    app.router.add_get(r"/previews/{suffix:.*}", get_preview_by_path)
    return app


def _stream_html_response(stream_path: str, topic: str) -> web.Response:
    """HTML page that auto-loads the MJPG stream for the topic."""
    mjpg_url = f"{stream_path.rstrip('/')}/mjpg"
    html = f"""<!DOCTYPE html>
<html>
<head><meta charset="utf-8"><title>Stream: {topic}</title></head>
<body>
<h1>Stream: {topic}</h1>
<img src="{mjpg_url}" alt="MJPG stream" style="max-width:100%;" />
</body>
</html>"""
    return web.Response(text=html, content_type="text/html")


def _preview_html_response(preview_path: str, topic: str) -> web.Response:
    """HTML page that auto-loads the static preview image for the topic."""
    image_url = f"{preview_path.rstrip('/')}/image.jpg"
    html = f"""<!DOCTYPE html>
<html>
<head><meta charset="utf-8"><title>Preview: {topic}</title></head>
<body>
<h1>Preview: {topic}</h1>
<img src="{image_url}" alt="Preview" style="max-width:100%;" />
</body>
</html>"""
    return web.Response(text=html, content_type="text/html")


async def _stream_mjpeg(request: web.Request, scraper: Any, topic: str) -> web.StreamResponse:
    """Stream MJPG multipart response; loop until client disconnects."""
    jpeg = scraper.get_latest_jpeg(topic)
    if jpeg is None:
        return web.json_response(
            {"error": "no image sample yet", "topic": topic},
            status=404,
            content_type="application/json",
        )
    response = web.StreamResponse(
        status=200,
        headers={
            "Content-Type": f"multipart/x-mixed-replace; boundary={MJPEG_BOUNDARY.decode()!s}",
            "Cache-Control": "no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0",
            "Pragma": "no-cache",
        },
    )
    await response.prepare(request)
    try:
        while True:
            jpeg = scraper.get_latest_jpeg(topic)
            if jpeg:
                part = (
                    b"--"
                    + MJPEG_BOUNDARY
                    + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode()
                    + b"\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )
                await response.write(part)
            await asyncio.sleep(MJPEG_STREAM_INTERVAL_S)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return response
