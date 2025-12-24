"""
Memory-efficient script to ingest the Physical AI textbook into Qdrant vector database
Processes content one small piece at a time to avoid memory issues
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

def chunk_text(text, chunk_size=300, overlap=30):  # Very small chunks to reduce memory usage
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

def process_single_chunk(chunk, file_path, title, chunk_idx):
    """
    Process a single chunk: create embedding and add to Qdrant
    """
    if len(chunk.strip()) <= 10:  # Skip very small chunks
        return False
        
    try:
        # Generate embedding using Cohere
        embeddings_response = cohere_client.embed(
            texts=[chunk],
            model=EMBEDDING_MODEL,
            input_type="search_document"
        )

        embedding = embeddings_response.embeddings[0]

        # Create point ID
        chunk_id = f"{os.path.basename(file_path)}_{chunk_idx:04d}"

        # Prepare point for Qdrant
        point = models.PointStruct(
            id=chunk_id,
            vector=embedding,
            payload={
                "content": chunk,
                "title": title,
                "path": file_path,
                "source_file": os.path.basename(file_path)
            }
        )

        # Upsert point to Qdrant
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=[point]
        )

        return True
    except Exception as e:
        logger.error(f"Error processing chunk {chunk_idx} of {file_path}: {e}")
        return False

def process_file(file_path):
    """
    Process a single markdown file, adding chunks one by one to Qdrant
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

        # Process each chunk individually to minimize memory usage
        processed_count = 0
        for i, chunk in enumerate(chunks):
            success = process_single_chunk(chunk, file_path, title, i)
            if success:
                processed_count += 1
                if processed_count % 10 == 0:  # Log progress every 10 chunks
                    logger.info(f"Processed {processed_count} chunks from {file_path}")

        logger.info(f"Completed processing {file_path}, added {processed_count} chunks")
        return processed_count

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
    for md_file in md_files[:5]:  # Limit to first 5 files to start with
        chunks_count = process_file(md_file)
        total_chunks += chunks_count
        logger.info(f"Completed processing {md_file}, total chunks so far: {total_chunks}")

    logger.info(f"Successfully ingested {total_chunks} total chunks into Qdrant")

if __name__ == "__main__":
    logger.info("Starting ingestion process...")
    ingest_documents()
    logger.info("Ingestion process completed!")