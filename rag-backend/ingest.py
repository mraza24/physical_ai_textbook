import uuid
import os
import time
from app.services.cohere_service import CohereService
from app.services.qdrant_service import QdrantService
from dotenv import load_dotenv

# .env file load karein
load_dotenv()

def run_ingestion(file_path: str):
    """Ek single file ko read karke database mein upload karne ka function"""
    cohere = CohereService()
    qdrant = QdrantService()

    # 1. Collection check/create
    try:
        qdrant.create_collection(vector_size=1024)
    except Exception:
        pass

    # 2. File reading
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            text_content = f.read()
    except Exception as e:
        print(f"❌ Error reading {file_path}: {e}")
        return

    if not text_content.strip():
        return

    # 3. Chunking (1000 chars)
    chunks = [text_content[i:i+1000] for i in range(0, len(text_content), 1000)]
    
    points = []
    print(f"⏳ Processing {len(chunks)} chunks from {os.path.basename(file_path)}...")

    for i, chunk in enumerate(chunks):
        if not chunk.strip(): continue
        
        success = False
        attempts = 0
        while not success and attempts < 3:
            try:
                # Embedding generate karein (search_document type)
                vector = cohere.embed_text(chunk, input_type="search_document")
                
                # Payload structure updated for better Citations
                points.append({
                    "id": str(uuid.uuid4()),
                    "vector": vector,
                    "payload": {
                        "text": chunk,
                        "source": os.path.basename(file_path),
                        "section_title": os.path.basename(file_path).replace('.md', '').replace('.txt', ''),
                        "deep_link_url": f"/docs/{os.path.basename(file_path).replace('.md', '')}",
                        "metadata": {
                            "chunk_index": i
                        }
                    }
                })
                success = True
                time.sleep(0.5) 
                
            except Exception as e:
                if "429" in str(e):
                    print(f"⚠️ Rate limit hit! Waiting 20s...")
                    time.sleep(20)
                    attempts += 1
                else:
                    print(f"❌ Error: {e}")
                    break

    # 4. Qdrant upload
    if points:
        try:
            qdrant.upsert_points(points)
            print(f"✅ Uploaded: {os.path.basename(file_path)}")
        except Exception as e:
            print(f"❌ Upload failed: {e}")

if __name__ == "__main__":
    # Apne folder ka path yahan check karein
    TARGET_FOLDER = "../textbook/textbook/docs" 
    
    if os.path.exists(TARGET_FOLDER):
        for root, dirs, files in os.walk(TARGET_FOLDER):
            for file in files:
                if file.endswith(".md") or file.endswith(".txt"):
                    file_path = os.path.join(root, file)
                    run_ingestion(file_path)
        print("\n✨ Mission Successful! All files processed.")
    else:
        print(f"❌ Folder not found: {TARGET_FOLDER}")