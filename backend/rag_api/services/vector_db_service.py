"""Vector database setup for Qdrant with textbook content indexing."""
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from typing import List, Dict, Any, Optional
import uuid
from ..config import Config
from ..utils.helpers import log_error, log_info
import os


class VectorDBService:
    """Service to handle vector database operations with Qdrant."""
    
    def __init__(self):
        # Initialize Qdrant client based on configuration
        if Config.QDRANT_HOST.startswith("http"):
            # If it's a URL, use it directly
            self.client = QdrantClient(
                url=Config.QDRANT_HOST,
                api_key=Config.QDRANT_API_KEY,
                prefer_grpc=True
            )
        else:
            # Otherwise assume it's a host for local connection
            self.client = QdrantClient(
                host=Config.QDRANT_HOST,
                port=6333,
                api_key=Config.QDRANT_API_KEY
            )
        
        self.collection_name = "textbook_content"
        self.dimension = Config.EMBEDDING_DIMENSION
    
    def initialize_collection(self):
        """Initialize the collection in Qdrant for storing textbook content."""
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)
            
            if not collection_exists:
                # Create collection with specified vector size
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.dimension,
                        distance=Distance.COSINE
                    )
                )
                
                log_info(f"Created Qdrant collection: {self.collection_name}", 
                        "VectorDBService.initialize_collection")
                
                # Create payload index for faster filtering
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="chapter_number",
                    field_schema=models.TextIndexParams(
                        type="text",
                        tokenizer=models.TokenizerType.WORD,
                        min_token_len=2,
                        max_token_len=10
                    )
                )
                
                # Create payload index for section number
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="section_number",
                    field_schema=models.TextIndexParams(
                        type="text",
                        tokenizer=models.TokenizerType.WORD,
                        min_token_len=1,
                        max_token_len=10
                    )
                )
            else:
                log_info(f"Qdrant collection {self.collection_name} already exists", 
                        "VectorDBService.initialize_collection")
            
            return True
            
        except Exception as e:
            log_error(e, "VectorDBService.initialize_collection")
            return False
    
    def index_content(self, content_id: str, content: str, 
                     embedding: List[float], metadata: Dict[str, Any]) -> bool:
        """Index a piece of content with its embedding."""
        try:
            points = [
                models.PointStruct(
                    id=content_id,
                    vector=embedding,
                    payload={
                        "content": content,
                        "metadata": metadata
                    }
                )
            ]
            
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            
            log_info(f"Indexed content with ID: {content_id}", 
                    "VectorDBService.index_content")
            return True
            
        except Exception as e:
            log_error(e, "VectorDBService.index_content")
            return False
    
    def search_content(self, query_embedding: List[float], 
                      limit: int = 5, filters: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """Search for relevant content based on the query embedding."""
        try:
            # Prepare filters if provided
            qdrant_filters = None
            if filters:
                filter_conditions = []
                
                # Add chapter filter if provided
                if "chapter_number" in filters:
                    filter_conditions.append(
                        models.FieldCondition(
                            key="metadata.chapter_number",
                            match=models.MatchValue(value=filters["chapter_number"])
                        )
                    )
                
                # Add section filter if provided
                if "section_number" in filters:
                    filter_conditions.append(
                        models.FieldCondition(
                            key="metadata.section_number", 
                            match=models.MatchValue(value=filters["section_number"])
                        )
                    )
                
                if filter_conditions:
                    qdrant_filters = models.Filter(
                        must=filter_conditions
                    )
            
            # Perform search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                query_filter=qdrant_filters
            )
            
            # Format results
            results = []
            for hit in search_results:
                results.append({
                    "id": hit.id,
                    "content": hit.payload.get("content", ""),
                    "metadata": hit.payload.get("metadata", {}),
                    "score": hit.score
                })
            
            log_info(f"Search completed with {len(results)} results", 
                    "VectorDBService.search_content")
            return results
            
        except Exception as e:
            log_error(e, "VectorDBService.search_content")
            return []
    
    def delete_content(self, content_id: str) -> bool:
        """Delete content from the vector database."""
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[content_id]
                )
            )
            
            log_info(f"Deleted content with ID: {content_id}", 
                    "VectorDBService.delete_content")
            return True
            
        except Exception as e:
            log_error(e, "VectorDBService.delete_content")
            return False
    
    def get_content_by_id(self, content_id: str) -> Optional[Dict[str, Any]]:
        """Retrieve a specific content by its ID."""
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[content_id]
            )
            
            if not records:
                return None
            
            record = records[0]
            return {
                "id": record.id,
                "content": record.payload.get("content", ""),
                "metadata": record.payload.get("metadata", {})
            }
            
        except Exception as e:
            log_error(e, "VectorDBService.get_content_by_id")
            return None


# Create a singleton instance
vector_db_service = VectorDBService()