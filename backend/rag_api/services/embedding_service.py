"""Embedding service for generating and processing text embeddings."""
from typing import List, Dict, Any
import openai
from ..config import Config
from ..utils.helpers import log_error, log_info
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Initialize OpenAI client
openai_client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))


class EmbeddingService:
    """Service to handle text embedding operations."""
    
    def __init__(self):
        self.client = openai_client
        self.model = "text-embedding-3-small"  # Using the recommended model
        self.dimension = Config.EMBEDDING_DIMENSION  # Expected dimension
    
    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate an embedding for the given text.
        
        Args:
            text: Input text to generate embedding for
            
        Returns:
            List of floats representing the embedding vector
        """
        try:
            log_info(f"Generating embedding for text of length {len(text)}", "EmbeddingService.generate_embedding")
            
            # Create embedding using OpenAI API
            response = self.client.embeddings.create(
                input=text,
                model=self.model
            )
            
            # Extract the embedding from the response
            embedding = response.data[0].embedding
            
            # Verify the embedding has the expected dimensions
            if len(embedding) != self.dimension:
                raise ValueError(f"Embedding dimension mismatch: expected {self.dimension}, got {len(embedding)}")
            
            return embedding
            
        except Exception as e:
            log_error(e, "EmbeddingService.generate_embedding")
            raise e
    
    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts.
        
        Args:
            texts: List of input texts to generate embeddings for
            
        Returns:
            List of embedding vectors
        """
        try:
            log_info(f"Generating embeddings for {len(texts)} texts", "EmbeddingService.generate_embeddings_batch")
            
            # Create embeddings using OpenAI API
            response = self.client.embeddings.create(
                input=texts,
                model=self.model
            )
            
            # Extract all embeddings from the response
            embeddings = [item.embedding for item in response.data]
            
            # Verify all embeddings have the expected dimensions
            for i, embedding in enumerate(embeddings):
                if len(embedding) != self.dimension:
                    raise ValueError(f"Embedding {i} dimension mismatch: expected {self.dimension}, got {len(embedding)}")
            
            return embeddings
            
        except Exception as e:
            log_error(e, "EmbeddingService.generate_embeddings_batch")
            raise e
    
    def calculate_similarity(self, embedding1: List[float], embedding2: List[float]) -> float:
        """
        Calculate cosine similarity between two embeddings.
        
        Args:
            embedding1: First embedding vector
            embedding2: Second embedding vector
            
        Returns:
            Cosine similarity score between -1 and 1
        """
        try:
            # Calculate dot product
            dot_product = sum(a * b for a, b in zip(embedding1, embedding2))
            
            # Calculate magnitudes
            magnitude1 = sum(a * a for a in embedding1) ** 0.5
            magnitude2 = sum(b * b for b in embedding2) ** 0.5
            
            # Calculate cosine similarity
            if magnitude1 == 0 or magnitude2 == 0:
                return 0.0
            
            similarity = dot_product / (magnitude1 * magnitude2)
            return similarity
            
        except Exception as e:
            log_error(e, "EmbeddingService.calculate_similarity")
            raise e


# Create a singleton instance
embedding_service = EmbeddingService()