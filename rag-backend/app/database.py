"""
Database connection management for Neon Postgres.
Provides asyncpg connection pool for async database operations.
"""
import os
from typing import Optional
import asyncpg
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Global connection pool
_pool: Optional[asyncpg.Pool] = None

async def get_database_pool() -> asyncpg.Pool:
    """
    Get or create the asyncpg connection pool.
    Returns the global connection pool, creating it if necessary.
    """
    global _pool

    if _pool is None:
        database_url = os.getenv("DATABASE_URL")

        if not database_url or database_url == "REPLACE_WITH_YOUR_NEON_POSTGRES_URL":
            raise ValueError(
                "DATABASE_URL not configured. "
                "Please update .env with your Neon Postgres connection string."
            )

        # Get pool configuration from environment
        min_size = int(os.getenv("DATABASE_POOL_MIN_SIZE", "5"))
        max_size = int(os.getenv("DATABASE_POOL_MAX_SIZE", "20"))

        print(f"Creating database pool (min: {min_size}, max: {max_size})...")

        _pool = await asyncpg.create_pool(
            database_url,
            min_size=min_size,
            max_size=max_size,
            command_timeout=60,
        )

        print("✅ Database pool created successfully")

    return _pool

async def close_database_pool():
    """Close the database connection pool."""
    global _pool

    if _pool is not None:
        await _pool.close()
        _pool = None
        print("✅ Database pool closed")

async def execute_query(query: str, *args):
    """
    Execute a database query using the connection pool.

    Args:
        query: SQL query string
        *args: Query parameters

    Returns:
        Query result
    """
    pool = await get_database_pool()
    async with pool.acquire() as connection:
        return await connection.fetch(query, *args)

async def execute_one(query: str, *args):
    """
    Execute a query and return a single row.

    Args:
        query: SQL query string
        *args: Query parameters

    Returns:
        Single row result or None
    """
    pool = await get_database_pool()
    async with pool.acquire() as connection:
        return await connection.fetchrow(query, *args)

async def execute_write(query: str, *args):
    """
    Execute a write query (INSERT, UPDATE, DELETE).

    Args:
        query: SQL query string
        *args: Query parameters

    Returns:
        Status of the execution
    """
    pool = await get_database_pool()
    async with pool.acquire() as connection:
        return await connection.execute(query, *args)
