"""Unit tests for RAG service configuration helpers."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from app.services.rag_service import RAGService


def test_openrouter_base_url_normalizes_completions_endpoint():
    """OpenRouter env vars sometimes include the full endpoint by mistake."""
    assert (
        RAGService._normalize_openrouter_base_url(
            "https://openrouter.ai/api/v1/chat/completions"
        )
        == "https://openrouter.ai/api/v1"
    )


def test_openrouter_base_url_keeps_valid_base_url():
    assert (
        RAGService._normalize_openrouter_base_url("https://openrouter.ai/api/v1/")
        == "https://openrouter.ai/api/v1"
    )


def test_openrouter_base_url_uses_default_for_empty_value():
    assert RAGService._normalize_openrouter_base_url(" ") == "https://openrouter.ai/api/v1"
