"""
Cohere API Service
Handles text embeddings and response generation using Latest Command-R Models (Dec 2025)
"""

import os
import logging
from typing import List
import cohere

logger = logging.getLogger(__name__)

class CohereService:
    """
    Service for interacting with Cohere API
    """

    def __init__(self):
        """
        Initialize Cohere client
        """
        self.api_key = os.getenv("COHERE_API_KEY")

        if not self.api_key:
            raise ValueError(
                "COHERE_API_KEY must be set in environment variables. "
                "Get your API key from https://dashboard.cohere.com and add it to .env"
            )

        try:
            self.client = cohere.Client(self.api_key, timeout=120) 
            logger.info("Cohere client initialized successfully with 120s timeout")
        except Exception as e:
            logger.error(f"Failed to initialize Cohere client: {e}")
            raise

        # --- MODEL CONFIGURATION UPDATED FOR DEC 2025 ---
        self.embedding_model = "embed-multilingual-v3.0"  
        # Purana 'command-r' retire ho chuka hai, ab stable version ye hai:
        self.generation_model = "command-r-08-2024"  
        self.temperature = float(os.getenv("TEMPERATURE_MIN", "0.3"))
        self.max_tokens = int(os.getenv("MAX_RESPONSE_TOKENS", "500"))

    def embed_text(self, text: str, input_type: str = "search_document") -> List[float]:
        """
        Generate embedding vector for text.
        """
        try:
            response = self.client.embed(
                texts=[text],
                model=self.embedding_model,
                input_type=input_type,
            )
            embedding = response.embeddings[0]
            return embedding
        except Exception as e:
            logger.error(f"Cohere embedding failed: {e}")
            raise

    def embed_batch(self, texts: List[str], batch_size: int = 96) -> List[List[float]]:
        """
        Generate embeddings for multiple texts.
        """
        try:
            all_embeddings = []
            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]
                response = self.client.embed(
                    texts=batch,
                    model=self.embedding_model,
                    input_type="search_document",
                )
                all_embeddings.extend(response.embeddings)
            return all_embeddings
        except Exception as e:
            logger.error(f"Cohere batch embedding failed: {e}")
            raise

    def generate_answer(
        self,
        query: str,
        context_chunks: List[str],
        temperature: float = None
    ) -> str:
        """
        Generates answer based on context using Chat API (Command-R 08-2024).
        """
        try:
            # Context formatting
            context_text = "\n\n".join([f"Document [{i+1}]: {chunk}" for i, chunk in enumerate(context_chunks)])

            # API call with updated model string
            response = self.client.chat(
                message=query,
                model=self.generation_model,
                preamble=f"""You are an expert physical AI textbook assistant. 
Answer the user's question based ONLY on the following textbook context. 
If the information is missing, say 'I cannot find sufficient information in the textbook.'

Context:
{context_text}""",
                temperature=temperature or self.temperature,
            )

            answer = response.text.strip()
            logger.info(f"Generated answer via {self.generation_model} (length={len(answer)})")
            return answer

        except Exception as e:
            logger.error(f"Cohere Chat API failed: {e}")
            raise

    def check_api_key(self) -> bool:
        """
        Verify that the API key is valid
        """
        try:
            self.client.embed(
                texts=["test"],
                model=self.embedding_model,
                input_type="search_query",
            )
            return True
        except Exception as e:
            logger.error(f"Cohere API key validation failed: {e}")
            return False