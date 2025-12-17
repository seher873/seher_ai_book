"""Validation functions for input data and business rules."""
import re
from typing import Optional
from datetime import datetime, timedelta
import uuid


def validate_session_token(session_token: str) -> bool:
    """Validate session token format."""
    if not session_token:
        return False
    
    # Check if it's a valid UUID string
    try:
        uuid.UUID(session_token)
        return True
    except ValueError:
        return False


def validate_query_content(query: str) -> tuple[bool, Optional[str]]:
    """Validate query content."""
    if not query or not query.strip():
        return False, "Query content cannot be empty"
    
    if len(query.strip()) < 1:
        return False, "Query must be at least 1 character long"
    
    if len(query) > 1000:
        return False, "Query must not exceed 1000 characters"
    
    return True, None


def validate_query_type(query_type: str) -> tuple[bool, Optional[str]]:
    """Validate query type."""
    valid_types = ["global", "focused"]
    
    if query_type not in valid_types:
        return False, f"Query type must be one of: {', '.join(valid_types)}"
    
    return True, None


def validate_chapter_section(chapter: str, section: str) -> tuple[bool, Optional[str]]:
    """Validate chapter and section format."""
    if not chapter or not chapter.strip():
        return False, "Chapter number is required"
    
    if not section or not section.strip():
        return False, "Section number is required"
    
    # Basic pattern for chapter.section (e.g. "1.2", "12.3")
    pattern = re.compile(r"^\d+(\.\d+)*$")  # e.g. "1", "1.2", "12.3.4", etc.
    
    if not pattern.match(chapter):
        return False, f"Invalid chapter format: {chapter}. Use format like '1' or '1.2'"
    
    if not pattern.match(section):
        return False, f"Invalid section format: {section}. Use format like '1' or '1.2'"
    
    return True, None


def validate_session_expiration(expires_at: datetime) -> tuple[bool, Optional[str]]:
    """Validate session expiration time."""
    if expires_at < datetime.utcnow():
        return False, "Session expiration time is in the past"
    
    # Maximum session duration: 24 hours
    max_duration = timedelta(hours=24)
    if (expires_at - datetime.utcnow()) > max_duration:
        return False, "Session duration exceeds maximum of 24 hours"
    
    return True, None


def validate_citation_data(citation_data: dict) -> tuple[bool, Optional[str]]:
    """Validate citation data."""
    required_fields = ["chapter_number", "section_number", "text_excerpt"]

    for field in required_fields:
        if field not in citation_data or not citation_data[field]:
            return False, f"Missing required field: {field}"

    is_valid, msg = validate_chapter_section(citation_data["chapter_number"], citation_data["section_number"])
    if not is_valid:
        return False, msg

    if len(citation_data["text_excerpt"]) < 5:
        return False, "Text excerpt is too short to be meaningful"

    # Validate URL if provided
    if "url" in citation_data and citation_data["url"]:
        url_valid, url_msg = validate_citation_url(citation_data["url"], citation_data["chapter_number"], citation_data["section_number"])
        if not url_valid:
            return False, url_msg

    return True, None


def validate_citation_url(url: str, chapter_number: str, section_number: str) -> tuple[bool, Optional[str]]:
    """Validate that the citation URL matches the chapter and section."""
    if not url:
        return False, "URL is required for citations"

    # Pattern for citation URL: /textbook/chapters/{chapter}/sections/{section}
    expected_pattern = f"/textbook/chapters/{chapter_number}/sections/{section_number}"

    if not url.endswith(expected_pattern):
        # Also check for URL-encoded versions
        import urllib.parse
        expected_encoded = urllib.parse.quote(expected_pattern, safe='/')
        if not url.endswith(expected_encoded):
            return False, f"URL doesn't match chapter.section: expected pattern ending with {expected_pattern}"

    # Validate URL format
    if not url.startswith('/') and not url.startswith('http'):
        return False, "URL must be a relative path starting with '/' or a full URL"

    return True, None