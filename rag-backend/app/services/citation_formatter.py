"""
Citation formatter service for formatting source attributions.

Handles:
- Grouping chunks by section
- Generating deep links
- Formatting citations for display
"""
from typing import List, Dict
from collections import defaultdict

from app.models.retrieved_chunk import RetrievedChunk
from app.models.citation import Citation


def format_citations(retrieved_chunks: List[RetrievedChunk]) -> List[Citation]:
    """
    Format citations from retrieved chunks.

    Groups chunks by section to avoid duplicate section citations,
    and generates clickable deep links.

    Args:
        retrieved_chunks: List of chunks used in the answer

    Returns:
        List of Citation objects
    """
    if not retrieved_chunks:
        return []

    # Group chunks by section (chapter + section)
    section_groups: Dict[str, List[RetrievedChunk]] = defaultdict(list)

    for chunk in retrieved_chunks:
        # Create section key from chapter and section
        section_key = f"{chunk.chapter} - {chunk.section}" if chunk.section else chunk.chapter
        section_groups[section_key].append(chunk)

    # Create citations
    citations = []

    for section_title, chunks in section_groups.items():
        # Use the deep link from the first chunk in this section
        deep_link_url = chunks[0].deep_link_url
        chunk_count = len(chunks)

        citation = Citation(
            section_title=section_title,
            deep_link_url=deep_link_url,
            chunk_count=chunk_count,
        )

        citations.append(citation)

    # Sort by chunk count (most referenced sections first)
    citations.sort(key=lambda c: c.chunk_count, reverse=True)

    return citations
