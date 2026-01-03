"""
Error Handling Middleware

Provides global exception handling for the FastAPI application.
Catches and formats errors from external services (Qdrant, Cohere, Gemini).
"""

from fastapi import Request, status
from fastapi.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
import logging
from typing import Callable

logger = logging.getLogger(__name__)


class ErrorHandlerMiddleware(BaseHTTPMiddleware):
    """
    Global error handling middleware.

    Catches exceptions from external services and returns user-friendly error responses.
    """

    async def dispatch(self, request: Request, call_next: Callable):
        """
        Process request and handle exceptions.

        Args:
            request: Incoming HTTP request
            call_next: Next middleware/handler in chain

        Returns:
            HTTP response (normal or error)
        """
        try:
            response = await call_next(request)
            return response

        except Exception as exc:
            # Log the full exception for debugging
            logger.error(f"Unhandled exception: {exc}", exc_info=True)

            # Determine error type and return appropriate response
            error_response = self._handle_exception(exc)
            return error_response

    def _handle_exception(self, exc: Exception) -> JSONResponse:
        """
        Convert exception to user-friendly JSON error response.

        Args:
            exc: The caught exception

        Returns:
            JSONResponse with appropriate status code and message
        """
        exc_str = str(exc).lower()
        exc_type = type(exc).__name__

        # Qdrant errors
        if "qdrant" in exc_str or "qdrantexception" in exc_type:
            # Check for specific filter/index errors
            if "index required" in exc_str or "bad request" in exc_str:
                return JSONResponse(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    content={
                        "error": "Invalid filter parameter",
                        "detail": "The requested filter is not supported. Try searching without lesson_id filter for now.",
                        "type": "filter_error"
                    }
                )
            return JSONResponse(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                content={
                    "error": "Vector search temporarily unavailable",
                    "detail": "The search service is experiencing issues. Please try again in a moment.",
                    "type": "qdrant_error"
                }
            )

        # Cohere errors
        if "cohere" in exc_str or "cohereexception" in exc_type or "toomanyrequests" in exc_str:
            if "429" in exc_str or "rate limit" in exc_str:
                return JSONResponse(
                    status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                    content={
                        "error": "Embedding service rate limit exceeded",
                        "detail": "Too many requests to the embedding service. Please wait a moment and try again.",
                        "type": "rate_limit_error"
                    }
                )
            else:
                return JSONResponse(
                    status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                    content={
                        "error": "Embedding service temporarily unavailable",
                        "detail": "The embedding service is experiencing issues. Please try again in a moment.",
                        "type": "cohere_error"
                    }
                )

        # Gemini / OpenAI API errors
        if "openai" in exc_str or "gemini" in exc_str or "api" in exc_str:
            if "429" in exc_str or "quota" in exc_str or "rate limit" in exc_str:
                return JSONResponse(
                    status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                    content={
                        "error": "Too many requests",
                        "detail": "Too many requests to the AI service. Please wait a moment and try again.",
                        "type": "rate_limit_error"
                    }
                )
            elif "503" in exc_str or "unavailable" in exc_str:
                return JSONResponse(
                    status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                    content={
                        "error": "AI service temporarily unavailable",
                        "detail": "The chatbot is temporarily unavailable. Please try again in a moment.",
                        "type": "llm_error"
                    }
                )
            else:
                return JSONResponse(
                    status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                    content={
                        "error": "AI processing error",
                        "detail": "An error occurred while processing your request. Please try again.",
                        "type": "llm_error"
                    }
                )

        # Database errors (Postgres)
        if "postgres" in exc_str or "database" in exc_str or "connection" in exc_str:
            return JSONResponse(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                content={
                    "error": "Database temporarily unavailable",
                    "detail": "The database service is experiencing issues. Your conversation will still work without persistence.",
                    "type": "database_error"
                }
            )

        # Generic server error
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={
                "error": "Internal server error",
                "detail": "An unexpected error occurred. Please try again later.",
                "type": "unknown_error"
            }
        )
