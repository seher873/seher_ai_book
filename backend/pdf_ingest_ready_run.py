import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from PyPDF2 import PdfReader
import re

# Load environment variables
load_dotenv()

# Initialize clients
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Create collection if not exists
try:
    qdrant_client.get_collection("book_docs")
    print("Collection 'book_docs' already exists")
except:
    qdrant_client.recreate_collection(
        collection_name="book_docs",
        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
    )
    print("Collection 'book_docs' created")

def split_text(text, max_words=300):
    """Split text into chunks of approximately max_words."""
    sentences = re.split(r'(?<=[.!?]) +', text)
    chunks = []
    current_chunk = []
    current_word_count = 0
    
    for sentence in sentences:
        sentence_word_count = len(sentence.split())
        
        if current_word_count + sentence_word_count <= max_words:
            current_chunk.append(sentence)
            current_word_count += sentence_word_count
        else:
            if current_chunk:
                chunks.append(' '.join(current_chunk))
            current_chunk = [sentence]
            current_word_count = sentence_word_count
    
    if current_chunk:
        chunks.append(' '.join(current_chunk))
    
    return chunks

def read_pdf(file_path):
    """Read text from PDF file."""
    reader = PdfReader(file_path)
    text_pages = []
    for page_num, page in enumerate(reader.pages):
        text = page.extract_text()
        text_pages.append((text, page_num + 1))
    return text_pages

def read_text_file(file_path):
    """Read text from plain text file."""
    with open(file_path, 'r', encoding='utf-8') as file:
        return [(file.read(), 1)]

def process_file(file_path):
    """Process either PDF or text file."""
    if file_path.lower().endswith('.pdf'):
        pages = read_pdf(file_path)
    else:
        pages = read_text_file(file_path)
    
    all_chunks = []
    for text, page_num in pages:
        chunks = split_text(text)
        for chunk in chunks:
            all_chunks.append((chunk, page_num))
    
    return all_chunks

# Example: Process a file
# Replace 'your_file.pdf' or 'your_file.txt' with your actual file
# file_chunks = process_file('your_file.pdf')  # or 'your_file.txt'

# For demonstration, using sample text
sample_text = "Artificial intelligence is transforming the world. Machine learning is a subset of AI. Deep learning uses neural networks with multiple layers. Natural language processing helps machines understand human language. Computer vision enables machines to interpret visual information. Robotics combines AI with physical systems. Reinforcement learning trains agents through rewards and punishments. Neural networks are inspired by the human brain. Transformers revolutionized natural language processing. Generative models can create new content. These technologies are advancing rapidly."
file_chunks = [(sample_text, 1)]  # Replace with actual file processing

# Create embeddings and prepare points
texts = [chunk for chunk, _ in file_chunks]
embeddings = cohere_client.embed(
    texts=texts,
    model="embed-english-v3.0",
    input_type="search_document"
).embeddings

points = []
for i, ((chunk, page_num), embedding) in enumerate(zip(file_chunks, embeddings)):
    point = models.PointStruct(
        id=i,
        vector=embedding,
        payload={"text": chunk, "page": page_num}
    )
    points.append(point)

# Upsert points
qdrant_client.upsert(
    collection_name="book_docs",
    points=points
)

# Print results
print(f"Number of points inserted: {len(points)}")
count = qdrant_client.count(collection_name="book_docs")
print(f"Total points in collection: {count.count}")