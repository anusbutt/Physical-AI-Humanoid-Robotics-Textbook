"""
Rate Limiting Middleware

Tracks requests by session (hashed User-Agent + IP) and enforces
per-hour limits. Returns HTTP 429 when exceeded.
"""

import hashlib
import time
import logging
from collections import defaultdict
from fastapi import Request, status
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from typing import Callable
import os

logger = logging.getLogger(__name__)

# Default: 100 requests/hour per session
DEFAULT_RATE_LIMIT = int(os.getenv("API_RATE_LIMIT", "100"))
WINDOW_SECONDS = 3600  # 1 hour


class RateLimitMiddleware(BaseHTTPMiddleware):
    """
    Per-session rate limiting middleware.

    Tracks requests by a hash of (IP + User-Agent) and enforces
    a configurable requests-per-hour limit.
    """

    def __init__(self, app, max_requests: int = DEFAULT_RATE_LIMIT):
        super().__init__(app)
        self.max_requests = max_requests
        # {session_hash: [timestamp1, timestamp2, ...]}
        self._requests: dict[str, list[float]] = defaultdict(list)
        self._last_cleanup = time.time()

    def _get_session_key(self, request: Request) -> str:
        """Generate a session key from IP + User-Agent."""
        client_ip = request.client.host if request.client else "unknown"
        user_agent = request.headers.get("user-agent", "unknown")
        raw = f"{client_ip}:{user_agent}"
        return hashlib.sha256(raw.encode()).hexdigest()[:16]

    def _cleanup_expired(self) -> None:
        """Remove entries older than the rate limit window. Runs at most once per minute."""
        now = time.time()
        if now - self._last_cleanup < 60:
            return
        self._last_cleanup = now
        cutoff = now - WINDOW_SECONDS
        expired_keys = []
        for key, timestamps in self._requests.items():
            self._requests[key] = [t for t in timestamps if t > cutoff]
            if not self._requests[key]:
                expired_keys.append(key)
        for key in expired_keys:
            del self._requests[key]

    async def dispatch(self, request: Request, call_next: Callable):
        # Only rate-limit the chat query endpoint
        if request.url.path != "/api/chat/query" or request.method == "OPTIONS":
            return await call_next(request)

        self._cleanup_expired()

        session_key = self._get_session_key(request)
        now = time.time()
        cutoff = now - WINDOW_SECONDS

        # Filter to only recent requests
        recent = [t for t in self._requests[session_key] if t > cutoff]
        self._requests[session_key] = recent

        if len(recent) >= self.max_requests:
            retry_after = int(WINDOW_SECONDS - (now - recent[0]))
            logger.warning(f"Rate limit exceeded for session {session_key} ({len(recent)} requests)")
            return JSONResponse(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                content={
                    "error": "Rate limit exceeded",
                    "detail": f"Maximum {self.max_requests} requests per hour. Please wait and try again.",
                    "type": "rate_limit_error"
                },
                headers={"Retry-After": str(max(retry_after, 1))}
            )

        # Record this request
        self._requests[session_key].append(now)
        return await call_next(request)
