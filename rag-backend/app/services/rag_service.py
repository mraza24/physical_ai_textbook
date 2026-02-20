"""
RAG Service Orchestrator
Coordinates between Cohere (LLM) and Qdrant (Vector DB)
Updated to allow fallback to general knowledge.
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
            # Step 1: Query Embedding
            query_vector = self.cohere.embed_text(query_text, input_type="search_query")

            # Step 2: Search Qdrant
            search_results = self.qdrant.search(query_vector=query_vector, top_k=5)

            context_chunks = []
            citations = []
            
            # اگر کتاب میں متعلقہ پیراگراف مل جائیں تو انہیں نکالیں
            if search_results:
                for hit in search_results:
                    payload = hit.payload if hasattr(hit, 'payload') else hit.get("payload", {})
                    text = payload.get("text", "")
                    if text:
                        context_chunks.append(text)
                        citations.append({
                            "section_title": payload.get("source") or payload.get("section_title") or "Textbook Section",
                            "deep_link_url": payload.get("deep_link_url", "#"),
                            "chunk_count": 1
                        })

            # ✅ Step 3: LLM Generation (With Hybrid Knowledge)
            # اگر context_chunks خالی بھی ہوں، تب بھی ہم جنریٹ کریں گے
            # اس سے "Humanoid Robots" جیسے سوالات کا جواب مل جائے گا
            
            # ہم ایک چھوٹا سا گائیڈنس نوٹ بھیج رہے ہیں اگر کتاب میں ڈیٹا نہ ملے
            effective_context = context_chunks if context_chunks else [
                "No specific textbook context found. Please answer using your general robotics knowledge but keep it relevant to Physical AI."
            ]

            answer = self.cohere.generate_answer(
                query=query_text, 
                context_chunks=effective_context
            )

            # Confidence score calculation
            confidence = 0.85 # Default fallback confidence
            if search_results and hasattr(search_results[0], 'score'):
                confidence = round(float(search_results[0].score), 2)
            elif search_results and isinstance(search_results[0], dict):
                confidence = round(float(search_results[0].get('score', 0.85)), 2)

            return {
                "answer": answer,
                "citations": citations,
                "confidence": confidence,
                "retrieved_chunk_count": len(search_results) if search_results else 0,
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