"""Citation model representing references to textbook content with navigation links."""
from sqlalchemy import Column, String, DateTime, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
import uuid
from datetime import datetime
from urllib.parse import quote
from ..db.connection import Base


class Citation(Base):
    __tablename__ = "citations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    response_id = Column(UUID(as_uuid=True), ForeignKey("responses.id"), nullable=False)
    chapter_number = Column(String(20), nullable=False)
    section_number = Column(String(20), nullable=False)
    text_excerpt = Column(Text, nullable=False)
    page_reference = Column(String(20))  # Optional page number
    url = Column(String)  # Direct link to the cited section in the textbook

    # Relationships
    response = relationship("Response", back_populates="citations")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if not self.id:
            self.id = uuid.uuid4()

        # Generate URL for navigation if not provided
        if not self.url and self.chapter_number and self.section_number:
            self.url = self.generate_navigation_url()

    def generate_navigation_url(self) -> str:
        """
        Generate a URL for navigating to the cited textbook section.
        """
        # URL format: /textbook/chapters/{chapter_number}/sections/{section_number}
        chapter_encoded = quote(self.chapter_number)
        section_encoded = quote(self.section_number)
        return f"/textbook/chapters/{chapter_encoded}/sections/{section_encoded}"

    def get_navigation_link_html(self) -> str:
        """
        Generate HTML for a clickable citation link.
        """
        if not self.url:
            self.url = self.generate_navigation_url()

        return f'<a href="{self.url}" data-chapter="{self.chapter_number}" data-section="{self.section_number}">Section {self.chapter_number}.{self.section_number}</a>'