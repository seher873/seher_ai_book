"""
Script to ingest markdown files from the Physical AI textbook into Qdrant vector database
"""
import os
import glob
import markdown
from bs4 import BeautifulSoup
import cohere
import qdrant_client
from qdrant_client.http import models
from dotenv import load_dotenv
import re
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize clients
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant_client = qdrant_client.QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Constants
COLLECTION_NAME = "seher_robotic_book_netlify_app"
EMBEDDING_MODEL = "embed-english-v3.0"
DOCS_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "docs")  # Path to the docs directory

def clean_markdown_content(content):
    """
    Clean and extract text content from markdown
    """
    # Convert markdown to HTML
    html = markdown.markdown(content)
    # Extract text from HTML
    soup = BeautifulSoup(html, 'html.parser')
    text = soup.get_text()
    # Remove extra whitespace
    text = re.sub(r'\s+', ' ', text).strip()
    return text

def chunk_text(text, chunk_size=1000, overlap=100):
    """
    Split text into overlapping chunks
    """
    chunks = []
    start = 0
    
    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        
        # Make sure we don't cut in the middle of a sentence if possible
        if end < len(text):
            # Look for sentence boundaries to avoid cutting mid-sentence
            last_period = chunk.rfind('.')
            last_exclamation = chunk.rfind('!')
            last_question = chunk.rfind('?')
            last_boundary = max(last_period, last_exclamation, last_question)
            
            if last_boundary > chunk_size // 2:  # Only adjust if the boundary is reasonably close
                chunk = text[start:last_boundary + 1]
                end = last_boundary + 1
        
        chunks.append(chunk)
        start = end - overlap  # Apply overlap for context continuity
        
        # If overlap adjustment made start >= len(text), break
        if start >= len(text):
            break
    
    return chunks

def create_collection():
    """
    Create Qdrant collection if it doesn't exist
    """
    try:
        # Check if collection exists
        collections = qdrant_client.get_collections().collections
        collection_names = [coll.name for coll in collections]
        
        if COLLECTION_NAME not in collection_names:
            # Create collection with appropriate vector size for Cohere embeddings
            # The embed-english-v3.0 model produces 1024-dim vectors for "search_document" input type
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
            )
            logger.info(f"Created collection: {COLLECTION_NAME}")
        else:
            logger.info(f"Collection {COLLECTION_NAME} already exists")
    except Exception as e:
        logger.error(f"Error creating collection: {e}")
        raise

def ingest_documents():
    """
    Process all markdown files in the docs directory and ingest them into Qdrant
    """
    # First, create the collection if it doesn't exist
    create_collection()
    
    # Find all markdown files in the docs directory
    md_files = glob.glob(os.path.join(DOCS_PATH, "**/*.md"), recursive=True)
    logger.info(f"Found {len(md_files)} markdown files to process")
    
    total_chunks = 0
    
    for md_file in md_files:
        logger.info(f"Processing file: {md_file}")
        
        try:
            # Read the markdown file
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Extract title from the markdown frontmatter or first heading
            title = "Untitled"
            lines = content.split('\n')
            for line in lines:
                if line.startswith('# '):  # First level heading
                    title = line[2:].strip()
                    break
                elif line.startswith('title:'):  # Frontmatter title
                    title = line[7:].strip()
                    break
            
            # Clean the content
            clean_content = clean_markdown_content(content)
            
            # Chunk the content
            chunks = chunk_text(clean_content)
            
            # Prepare for embedding
            texts_to_embed = []
            chunk_info = []
            
            for i, chunk in enumerate(chunks):
                if len(chunk.strip()) > 10:  # Skip very small chunks
                    texts_to_embed.append(chunk)
                    # Create a unique ID for each chunk
                    chunk_id = f"{os.path.basename(md_file)}_{i:03d}"
                    chunk_info.append({
                        "id": chunk_id,
                        "title": title,
                        "path": md_file,
                        "content": chunk
                    })
            
            if not texts_to_embed:
                logger.warning(f"No valid chunks found for {md_file}")
                continue
            
            logger.info(f"Generating embeddings for {len(texts_to_embed)} chunks from {md_file}")
            
            # Generate embeddings using Cohere
            embeddings_response = cohere_client.embed(
                texts=texts_to_embed,
                model=EMBEDDING_MODEL,
                input_type="search_document"
            )
            
            embeddings = embeddings_response.embeddings
            
            # Prepare points for Qdrant
            points = []
            for i, info in enumerate(chunk_info):
                points.append(models.PointStruct(
                    id=info["id"],
                    vector=embeddings[i],
                    payload={
                        "content": info["content"],
                        "title": info["title"],
                        "path": info["path"],
                        "source_file": os.path.basename(md_file)
                    }
                ))
            
            # Upsert points to Qdrant
            qdrant_client.upsert(
                collection_name=COLLECTION_NAME,
                points=points
            )
            
            total_chunks += len(points)
            logger.info(f"Ingested {len(points)} chunks from {md_file}")
            
        except Exception as e:
            logger.error(f"Error processing file {md_file}: {e}")
    
    logger.info(f"Successfully ingested {total_chunks} total chunks into Qdrant")

if __name__ == "__main__":
    logger.info("Starting ingestion process...")
    ingest_documents()
    logger.info("Ingestion process completed!")