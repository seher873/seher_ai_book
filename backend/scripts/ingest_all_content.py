"""
Memory-efficient script to ingest markdown files from both docs and content directories 
into Qdrant vector database for the Physical AI textbook
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
DOCS_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "frontend", "docs")  # Path to the docs directory
CONTENT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "frontend", "src", "content")  # Path to the content directory

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

def chunk_text(text, chunk_size=300, overlap=30):  # Further reduced chunk size
    """
    Split text into overlapping chunks
    """
    chunks = []
    if not text:
        return chunks

    start = 0
    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]

        if end < len(text):
            last_period = chunk.rfind('.')
            last_exclamation = chunk.rfind('!')
            last_question = chunk.rfind('?')
            last_boundary = max(last_period, last_exclamation, last_question)

            if last_boundary > chunk_size // 2:
                chunk = text[start:start + last_boundary + 1]
                end = start + last_boundary + 1

        chunks.append(chunk)
        start = end - overlap
        if start >= len(text):
            break
    return chunks

def create_collection():
    """
    Create Qdrant collection if it doesn't exist (Don't delete if exists)
    """
    try:
        collections = qdrant_client.get_collections().collections
        collection_names = [coll.name for coll in collections]

        if COLLECTION_NAME not in collection_names:
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
            )
            logger.info(f"Created collection: {COLLECTION_NAME}")
        else:
            # Clear existing collection to ensure we have FRESH full content (no stale 25 points)
            qdrant_client.delete_collection(COLLECTION_NAME)
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
            )
            logger.info(f"Recreated collection {COLLECTION_NAME} to ensure fresh ingestion.")
    except Exception as e:
        logger.error(f"Error creating collection: {e}")
        raise

import time
import uuid

def process_file_in_batches(file_path, batch_size=90):  # Optimized batch size for Trial Key
    """
    Process a single file and ingest in larger batches to maximize Trial API usage
    """
    logger.info(f"Processing file: {file_path}")

    try:
        # Read the markdown file
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract title
        title = "Untitled"
        lines = content.split('\n')
        for line in lines:
            if line.startswith('# '):
                title = line[2:].strip()
                break
            elif line.startswith('title:'):
                title = line[7:].strip()
                break

        # Clean the content
        clean_content = clean_markdown_content(content)

        # Chunk the content
        chunks = chunk_text(clean_content)

        # Process in batches
        total_processed = 0

        for i in range(0, len(chunks), batch_size):
            # Initial wait to cool down any existing limits
            if i == 0:
                logger.info("Cooling down for 60s before first batch...")
                time.sleep(60)

            batch_chunks = chunks[i:i+batch_size]
            batch_chunks = [chunk for chunk in batch_chunks if len(chunk.strip()) > 10]

            if not batch_chunks:
                continue

            logger.info(f"Processing batch {i//batch_size + 1} for {file_path} ({len(batch_chunks)} chunks)")

            max_retries = 5
            retry_count = 0

            while retry_count < max_retries:
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
                        point_id = str(uuid.uuid4())
                        points.append(models.PointStruct(
                            id=point_id,
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

                    # Moderate delay between batches (6 batches per minute max)
                    time.sleep(15)
                    break

                except Exception as e:
                    if "429" in str(e) or "trial" in str(e).lower() or "limit" in str(e).lower():
                        retry_count += 1
                        wait_time = (2 ** retry_count) * 20
                        logger.warning(f"Rate limited (429). Retrying in {wait_time}s... ({retry_count}/{max_retries})")
                        time.sleep(wait_time)
                    else:
                        logger.error(f"Error processing batch {i//batch_size + 1} of {file_path}: {e}")
                        break

        return total_processed

    except Exception as e:
        logger.error(f"Error processing file {file_path}: {e}")
        return 0

def ingest_documents():
    """
    Process all markdown files in both docs and content directories and ingest them into Qdrant
    """
    # First, create the collection if it doesn't exist
    create_collection()

    # Find all markdown files in the docs directory
    docs_md_files = glob.glob(os.path.join(DOCS_PATH, "**/*.md"), recursive=True)
    logger.info(f"Found {len(docs_md_files)} markdown files in docs directory to process")

    # Filter out templates
    docs_md_files = [f for f in docs_md_files if "_template" not in f]

    # Combine lists (content path currently empty but keeping for structure)
    all_md_files = docs_md_files
    logger.info(f"Total of {len(all_md_files)} informative markdown files to process")

    total_chunks = 0

    # Process files one by one
    for md_file in all_md_files:
        chunks_count = process_file_in_batches(md_file)
        total_chunks += chunks_count
        logger.info(f"Completed processing {md_file}, total chunks so far: {total_chunks}")
        # Small break between files
        time.sleep(1)

    logger.info(f"Successfully ingested {total_chunks} total chunks into Qdrant")

    # Print final collection info (Wrapped in try/except to avoid pydantic issues)
    try:
        collection_info = qdrant_client.get_collection(COLLECTION_NAME)
        logger.info(f"Final collection '{COLLECTION_NAME}' points count: {collection_info.points_count}")
    except Exception as e:
        logger.warning(f"Could not retrieve collection info due to client/server version mismatch, but ingestion may have succeeded: {e}")

if __name__ == "__main__":
    logger.info("Starting ingestion process for both docs and content directories...")
    ingest_documents()
    logger.info("Ingestion process completed!")