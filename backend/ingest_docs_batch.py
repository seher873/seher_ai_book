"""
Memory-efficient script to ingest markdown files from the Physical AI textbook into Qdrant vector database
Processes files individually to minimize memory usage
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
    api_key=os.getenv("QDRANT_API_KEY"),
    timeout=60  # Increase timeout for cloud operations
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

def chunk_text(text, chunk_size=500, overlap=50):  # Smaller chunks to reduce memory usage
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
            # Clear existing collection to start fresh
            qdrant_client.delete_collection(COLLECTION_NAME)
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
            )
            logger.info(f"Recreated collection: {COLLECTION_NAME}")
    except Exception as e:
        logger.error(f"Error creating collection: {e}")
        raise

def process_file_in_batches(file_path, batch_size=10):
    """
    Process a single file and ingest in small batches to manage memory
    """
    logger.info(f"Processing file: {file_path}")

    try:
        # Read the markdown file
        with open(file_path, 'r', encoding='utf-8') as f:
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

        # Process in batches
        total_processed = 0
        
        for i in range(0, len(chunks), batch_size):
            batch_chunks = chunks[i:i+batch_size]
            
            # Filter out very small chunks
            batch_chunks = [chunk for chunk in batch_chunks if len(chunk.strip()) > 10]
            
            if not batch_chunks:
                continue
                
            logger.info(f"Processing batch {i//batch_size + 1} for {file_path} ({len(batch_chunks)} chunks)")
            
            try:
                # Generate embeddings using Cohere
                embeddings_response = cohere_client.embed(
                    texts=batch_chunks,
                    model=EMBEDDING_MODEL,
                    input_type="search_document"
                )

                embeddings = embeddings_response.embeddings

                # Prepare points for Qdrant
                points = []
                for j, (chunk, embedding) in enumerate(zip(batch_chunks, embeddings)):
                    chunk_id = f"{os.path.basename(file_path)}_{i+j:04d}"
                    points.append(models.PointStruct(
                        id=chunk_id,
                        vector=embedding,
                        payload={
                            "content": chunk,
                            "title": title,
                            "path": file_path,
                            "source_file": os.path.basename(file_path)
                        }
                    ))

                # Upsert points to Qdrant
                qdrant_client.upsert(
                    collection_name=COLLECTION_NAME,
                    points=points
                )

                total_processed += len(points)
                logger.info(f"Ingested {len(points)} chunks from batch {i//batch_size + 1} of {file_path}")

            except Exception as e:
                logger.error(f"Error processing batch {i//batch_size + 1} of {file_path}: {e}")
                continue  # Continue with the next batch even if one fails

        return total_processed

    except Exception as e:
        logger.error(f"Error processing file {file_path}: {e}")
        return 0

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

    # Process files one by one to manage memory usage
    for md_file in md_files:
        chunks_count = process_file_in_batches(md_file)
        total_chunks += chunks_count
        logger.info(f"Completed processing {md_file}, total chunks so far: {total_chunks}")

    logger.info(f"Successfully ingested {total_chunks} total chunks into Qdrant")

if __name__ == "__main__":
    logger.info("Starting ingestion process...")
    ingest_documents()
    logger.info("Ingestion process completed!")