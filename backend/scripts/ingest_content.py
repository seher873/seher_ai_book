"""
Script to ingest textbook content into the Qdrant vector database
"""
import os
import sys
from pathlib import Path
import markdown
from bs4 import BeautifulSoup
import cohere
import qdrant_client
from qdrant_client.http import models
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize clients
co = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant = qdrant_client.QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

def extract_text_from_markdown(file_path):
    """Extract text content from a Markdown file"""
    with open(file_path, 'r', encoding='utf-8') as file:
        md_content = file.read()
    
    # Convert markdown to HTML
    html_content = markdown.markdown(md_content)
    
    # Extract text from HTML
    soup = BeautifulSoup(html_content, 'html.parser')
    text = soup.get_text()
    
    # Clean up text (remove extra whitespace)
    text = ' '.join(text.split())
    
    return text

def chunk_text(text, chunk_size=1000, overlap=100):
    """Split text into overlapping chunks"""
    chunks = []
    start = 0
    
    while start < len(text):
        end = start + chunk_size
        
        # If this isn't the last chunk, try to break at a sentence boundary
        if end < len(text):
            # Look for a sentence boundary near the end
            temp_chunk = text[start:end]
            last_period = temp_chunk.rfind('. ')
            if last_period != -1 and last_period > chunk_size // 2:  # Only if it's reasonably far in
                end = start + last_period + 2
        
        chunk = text[start:end]
        chunks.append(chunk)
        
        # Move start forward, accounting for overlap
        start = end - overlap if end < len(text) else end
        
        # If our chunk is still too large, just split at the limit
        if len(chunk) >= chunk_size:
            start = end
    
    return chunks

def process_textbook_content(docs_dir):
    """Process all textbook content in the docs directory"""
    documents = []
    
    docs_path = Path(docs_dir)
    
    # Walk through all markdown files in the docs directory
    for md_file in docs_path.rglob("*.md"):
        relative_path = md_file.relative_to(docs_path)
        path_str = str(relative_path).replace('.md', '')
        
        # Extract content from the markdown file
        content = extract_text_from_markdown(md_file)
        
        # Create title from filename/path
        title = path_str.replace('-', ' ').replace('/', ' - ').title()
        
        # Chunk the content
        chunks = chunk_text(content)
        
        # Create document entries for each chunk
        for i, chunk in enumerate(chunks):
            if chunk.strip():  # Only add non-empty chunks
                doc = {
                    "title": f"{title} (Part {i+1})",
                    "content": chunk,
                    "path": path_str,
                    "source_file": str(md_file)
                }
                documents.append(doc)
    
    return documents

def ingest_documents(documents):
    """Ingest documents into Qdrant"""
    if not documents:
        print("No documents to ingest")
        return
    
    print(f"Processing {len(documents)} document chunks...")
    
    # Prepare texts for embedding
    texts = [doc["content"] for doc in documents]
    
    # Generate embeddings
    print("Generating embeddings...")
    response = co.embed(
        texts=texts,
        model="embed-multilingual-v3.0"
    )
    embeddings = response.embeddings
    
    # Prepare points for Qdrant
    points = []
    for i, (doc, embedding) in enumerate(zip(documents, embeddings)):
        point = models.PointStruct(
            id=i,
            vector=embedding,
            payload={
                "title": doc["title"],
                "content": doc["content"],
                "path": doc["path"],
                "source_file": doc["source_file"]
            }
        )
        points.append(point)
    
    # Create or recreate the collection in Qdrant
    print("Creating Qdrant collection...")
    qdrant.recreate_collection(
        collection_name="textbook_content",
        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
    )
    
    # Upload points to Qdrant
    print("Uploading to Qdrant...")
    qdrant.upload_points(
        collection_name="textbook_content",
        points=points
    )
    
    print(f"Successfully ingested {len(points)} document chunks into Qdrant")

def main():
    # Get the docs directory from command line argument or use default
    docs_dir = sys.argv[1] if len(sys.argv) > 1 else "../../frontend/docs"
    
    if not os.path.exists(docs_dir):
        print(f"Error: Docs directory '{docs_dir}' does not exist")
        sys.exit(1)
    
    # Process textbook content
    documents = process_textbook_content(docs_dir)
    
    # Ingest documents into Qdrant
    ingest_documents(documents)
    
    print("Ingestion complete!")

if __name__ == "__main__":
    main()