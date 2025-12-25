"""
RAG Service Orchestrator
Coordinates between Cohere (LLM) and Qdrant (Vector DB)
"""

import logging
import time
from typing import List, Dict, Any
from app.services.cohere_service import CohereService
from app.services.qdrant_service import QdrantService

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        try:
            self.cohere = CohereService()
            self.qdrant = QdrantService()
            logger.info("RAGService: Cohere and Qdrant services initialized")
        except Exception as e:
            logger.error(f"RAGService initialization failed: {e}")
            raise

    async def query(self, query_text: str, selected_text: str = None) -> Dict[str, Any]:
        start_time = time.time()
        try:
            # Step 1: Query Embedding (input_type="search_query" is must for Cohere v3)
            query_vector = self.cohere.embed_text(query_text, input_type="search_query")

            # Step 2: Search Qdrant
            search_results = self.qdrant.search(query_vector=query_vector, top_k=5)

            # Agar kuch nahi mila
            if not search_results:
                logger.warning(f"No relevant chunks found for: {query_text}")
                return {
                    "answer": "I'm sorry, I couldn't find any relevant information in the textbook to answer your question.",
                    "citations": [],
                    "confidence": 0.0,
                    "retrieved_chunk_count": 0,
                    "processing_time_ms": int((time.time() - start_time) * 1000)
                }

            context_chunks = []
            citations = []
            
            for hit in search_results:
                # Support both dict and object formats from Qdrant
                payload = hit.payload if hasattr(hit, 'payload') else hit.get("payload", {})
                score = hit.score if hasattr(hit, 'score') else hit.get("score", 0.0)
                
                text = payload.get("text", "")
                if text:
                    context_chunks.append(text)
                    citations.append({
                        "section_title": payload.get("source") or payload.get("section_title") or "Textbook Section",
                        "deep_link_url": payload.get("deep_link_url", "#"),
                        "chunk_count": 1
                    })

            # Step 3: LLM Generation
            if context_chunks:
                answer = self.cohere.generate_answer(query=query_text, context_chunks=context_chunks)
            else:
                answer = "Context chunks were empty. Cannot generate answer."

            return {
                "answer": answer,
                "citations": citations,
                "confidence": round(float(search_results[0].score if hasattr(search_results[0], 'score') else search_results[0].get('score', 0.85)), 2),
                "retrieved_chunk_count": len(search_results),
                "processing_time_ms": int((time.time() - start_time) * 1000)
            }

        except Exception as e:
            logger.error(f"Error in RAG query pipeline: {e}")
            raise

    async def health_check(self) -> Dict[str, Any]:
        cohere_ok = self.cohere.check_api_key()
        return {
            "cohere": cohere_ok,
            "qdrant": True,
            "overall": cohere_ok
        }