-- RAG Chatbot Database Schema
-- Migration 001: Initialize core tables for query tracking, responses, and book metadata

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- Queries table: stores all user queries
CREATE TABLE IF NOT EXISTS queries (
    query_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    session_id UUID NOT NULL,
    query_text TEXT NOT NULL,
    selected_text TEXT,
    timestamp TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Index for session-based queries
CREATE INDEX IF NOT EXISTS idx_queries_session_id ON queries(session_id);
CREATE INDEX IF NOT EXISTS idx_queries_timestamp ON queries(timestamp);

-- Responses table: stores chatbot responses for each query
CREATE TABLE IF NOT EXISTS responses (
    response_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    query_id UUID NOT NULL REFERENCES queries(query_id) ON DELETE CASCADE,
    response_text TEXT NOT NULL,
    token_count INT NOT NULL,
    source_references JSONB,
    confidence_score FLOAT,
    generation_temperature FLOAT,
    timestamp TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Index for query-based responses
CREATE INDEX IF NOT EXISTS idx_responses_query_id ON responses(query_id);
CREATE INDEX IF NOT EXISTS idx_responses_timestamp ON responses(timestamp);

-- Sessions table: tracks user sessions
CREATE TABLE IF NOT EXISTS sessions (
    session_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    last_active_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    query_count INT NOT NULL DEFAULT 0
);

-- Index for session activity tracking
CREATE INDEX IF NOT EXISTS idx_sessions_created_at ON sessions(created_at);
CREATE INDEX IF NOT EXISTS idx_sessions_last_active ON sessions(last_active_at);

-- Book metadata table: stores version and structure information
CREATE TABLE IF NOT EXISTS book_metadata (
    content_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    book_version VARCHAR(50) NOT NULL,
    chapter_title VARCHAR(255),
    section_title VARCHAR(255),
    last_updated TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Index for version and content lookup
CREATE INDEX IF NOT EXISTS idx_book_metadata_version ON book_metadata(book_version);
CREATE INDEX IF NOT EXISTS idx_book_metadata_chapter ON book_metadata(chapter_title);

-- Ingestion logs table: tracks content ingestion operations
CREATE TABLE IF NOT EXISTS ingestion_logs (
    ingestion_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    timestamp TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    chunks_processed INT NOT NULL DEFAULT 0,
    errors TEXT,
    status VARCHAR(50) NOT NULL
);

-- Index for ingestion tracking
CREATE INDEX IF NOT EXISTS idx_ingestion_logs_timestamp ON ingestion_logs(timestamp);
CREATE INDEX IF NOT EXISTS idx_ingestion_logs_status ON ingestion_logs(status);
