#!/usr/bin/env python3
"""
Migration runner for RAG Chatbot database schema.
Runs SQL migration files against the Neon Postgres database.
"""
import os
import sys
from pathlib import Path
import psycopg2
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def run_migration(migration_file: Path) -> None:
    """Run a single migration file against the database."""
    database_url = os.getenv("DATABASE_URL")

    if not database_url or database_url == "REPLACE_WITH_YOUR_NEON_POSTGRES_URL":
        print("❌ ERROR: DATABASE_URL not configured in .env file")
        print("Please update .env with your actual Neon Postgres connection string")
        sys.exit(1)

    print(f"Running migration: {migration_file.name}")

    try:
        # Connect to database
        conn = psycopg2.connect(database_url)
        cursor = conn.cursor()

        # Read and execute migration file
        with open(migration_file, 'r') as f:
            sql = f.read()

        cursor.execute(sql)
        conn.commit()

        print(f"✅ Migration {migration_file.name} completed successfully")

        cursor.close()
        conn.close()

    except psycopg2.Error as e:
        print(f"❌ Database error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)

def main():
    """Run all migrations in the migrations/ directory."""
    # Get migrations directory
    migrations_dir = Path(__file__).parent.parent / "migrations"

    if not migrations_dir.exists():
        print(f"❌ Migrations directory not found: {migrations_dir}")
        sys.exit(1)

    # Get all .sql files sorted by name
    migration_files = sorted(migrations_dir.glob("*.sql"))

    if not migration_files:
        print("No migration files found")
        return

    print(f"Found {len(migration_files)} migration(s) to run\n")

    # Run each migration
    for migration_file in migration_files:
        run_migration(migration_file)
        print()

    print("🎉 All migrations completed successfully!")

if __name__ == "__main__":
    main()
