"""RAG service for processing queries and generating responses with conversation context."""
from typing import Dict, List, Any
import openai
from ..config import Config
from ..services.content_filter import content_filter_service
from ..services.vector_db_service import vector_db_service
from ..utils.helpers import log_error, log_info
from ..utils.validators import validate_chapter_section
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Initialize OpenAI client
openai_client = openai.OpenAI(api_key=os.getenv("OPENAI_API_KEY"))


class RAGService:
    """Service to handle RAG (Retrieval-Augmented Generation) operations with conversation context."""

    def __init__(self):
        self.client = openai_client
        self.system_prompt = """
        You are an AI assistant for the Physical AI textbook. Your role is to provide accurate answers
        based solely on the content provided in the textbook. Only use information from the provided
        context. Do not hallucinate or provide information outside of the textbook content.
        Always provide citations to specific chapters and sections in the format
        [Chapter X, Section Y.Z]. If the question cannot be answered based on the provided context,
        clearly state that.
        """

    async def process_query(
        self,
        query_content: str,
        session_token: str,
        query_type: str = "global",
        selected_text: str = None,
        conversation_history: List[Dict] = None
    ) -> Dict[str, Any]:
        """
        Process a user query through the RAG pipeline with conversation context.

        Args:
            query_content: The user's question
            session_token: Session identifier
            query_type: Either "global" (search entire corpus) or "focused" (search selected text)
            selected_text: Text selected by user for focused queries
            conversation_history: Previous query-response pairs for context

        Returns:
            Dictionary containing response and citations
        """
        log_info(f"Processing query: {query_content[:50]}...", "RAGService.process_query")

        try:
            # Format conversation history as context if available
            history_context = ""
            if conversation_history:
                history_context = "Previous conversation context:\n"
                for i, exchange in enumerate(conversation_history[-5:]):  # Use last 5 exchanges
                    history_context += f"Q{i+1}: {exchange.get('query', '')}\n"
                    history_context += f"A{i+1}: {exchange.get('response', '')}\n\n"

            # Determine content context based on query type
            if query_type == "focused" and selected_text:
                # For focused queries, use selected text as context
                context_chunks = [{"content": selected_text, "metadata": {"chapter_number": "N/A", "section_number": "N/A"}}]
            else:
                # For global queries, search the vector database
                # First, we need to get the embedding for the query
                query_embedding_response = self.client.embeddings.create(
                    input=query_content,
                    model="text-embedding-3-small"  # Using the recommended model
                )
                query_embedding = query_embedding_response.data[0].embedding

                # Search for relevant content in the vector database
                context_chunks = vector_db_service.search_content(
                    query_embedding=query_embedding,
                    limit=5  # Get top 5 most relevant chunks
                )

            # Format content context for the LLM
            content_context = ""
            citations = []

            for chunk in context_chunks:
                content_context += f"\n---\n{chunk['content']}\n---\n"

                # Extract citation info if available
                metadata = chunk.get('metadata', {})
                chapter_number = metadata.get('chapter_number', 'Unknown')
                section_number = metadata.get('section_number', 'Unknown')

                # Verify the citation data is valid
                is_valid, _ = validate_chapter_section(chapter_number, section_number)
                if is_valid:
                    citations.append({
                        "chapter_number": chapter_number,
                        "section_number": section_number,
                        "text_excerpt": chunk['content'][:200] + "..." if len(chunk['content']) > 200 else chunk['content'],
                        "url": f"/docs/physical-ai-textbook/chapter{chapter_number}#section{section_number}"
                    })

            # Construct the full prompt for the LLM with both history and content context
            full_prompt = f"""
            {history_context}

            Content Context: {content_context}

            Current Question: {query_content}

            Please provide a detailed response based on the provided context and conversation history.
            Include specific citations to chapters and sections where the information is found.
            If the question refers to previous exchanges, ensure your response is consistent with the conversation flow.
            """

            # Call the OpenAI API to generate a response
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",  # or "gpt-4" if preferred
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": full_prompt}
                ],
                max_tokens=Config.MAX_RESPONSE_TOKENS,
                temperature=0.3  # Lower temperature for more factual responses
            )

            # Extract the generated text
            generated_text = response.choices[0].message.content.strip()

            log_info(f"Generated response of {len(generated_text)} characters", "RAGService.process_query")

            return {
                "response": generated_text,
                "citations": citations,
                "query_type": query_type,
                "session_token": session_token
            }

        except Exception as e:
            log_error(e, "RAGService.process_query")
            raise e


# Create a singleton instance
rag_service = RAGService()