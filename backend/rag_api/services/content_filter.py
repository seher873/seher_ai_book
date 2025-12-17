"""Content filtering service using OpenAI Moderation API."""
import openai
from typing import Dict, List, Any
from ..utils.helpers import log_error, log_info
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Initialize OpenAI client
openai.api_key = os.getenv("OPENAI_API_KEY")


class ContentFilterService:
    """Service to filter content using OpenAI's Moderation API."""
    
    def __init__(self):
        self.client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
    
    def validate_content(self, content: str, content_type: str = "query") -> Dict[str, Any]:
        """
        Validate content using OpenAI Moderation API.
        
        Args:
            content: The content to validate
            content_type: Type of content ("query" or "response")
            
        Returns:
            Dictionary with validation results
        """
        try:
            log_info(f"Validating {content_type} content", "ContentFilterService.validate_content")
            
            # Call OpenAI Moderation API
            response = self.client.moderations.create(input=content)
            moderation_result = response.results[0]
            
            # Process the moderation result
            is_approved = not moderation_result.flagged
            moderation_flags = []
            
            if moderation_result.flagged:
                # Collect all categories that triggered the flag
                for category, flagged in moderation_result.categories.model_dump().items():
                    if flagged:
                        moderation_flags.append(category)
            
            log_info(f"Content validation result: approved={is_approved}, flags={moderation_flags}", 
                    "ContentFilterService.validate_content")
            
            return {
                "is_approved": is_approved,
                "reason": "Content flagged by moderation system" if not is_approved else None,
                "moderation_flags": moderation_flags,
                "content_type": content_type
            }
            
        except Exception as e:
            log_error(e, "ContentFilterService.validate_content")
            return {
                "is_approved": False,
                "reason": f"Content validation failed with error: {str(e)}",
                "moderation_flags": ["validation_error"],
                "content_type": content_type
            }
    
    def filter_query(self, query: str) -> Dict[str, Any]:
        """Filter a user query before processing."""
        return self.validate_content(query, "query")
    
    def filter_response(self, response: str) -> Dict[str, Any]:
        """Filter a response before sending to user."""
        return self.validate_content(response, "response")


# Singleton instance
content_filter_service = ContentFilterService()