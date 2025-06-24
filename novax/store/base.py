"""
Base database connection and shared utilities for the store package.
"""

import sqlite3
from typing import Dict
from contextlib import contextmanager
from ..interfaces import DatabaseConnectionInterface


class DatabaseConnection(DatabaseConnectionInterface):
    """Shared database connection and initialization"""
    
    def __init__(self, db_path: str = "nova_programs.db"):
        self.db_path = db_path
        self.init_database()
    
    def init_database(self):
        """Initialize the database and create tables if they don't exist"""
        with self.get_connection() as conn:
            cursor = conn.cursor()
            
            # Create program_templates table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS program_templates (
                    name TEXT PRIMARY KEY,
                    model_class_name TEXT NOT NULL,
                    schema TEXT NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            """)
            
            # Create program_instances table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS program_instances (
                    name TEXT PRIMARY KEY,
                    template_name TEXT NOT NULL,
                    data TEXT NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    FOREIGN KEY (template_name) REFERENCES program_templates (name)
                )
            """)
            
            # Create program_runs table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS program_runs (
                    run_id TEXT PRIMARY KEY,
                    program_name TEXT NOT NULL,
                    status TEXT NOT NULL DEFAULT 'pending',
                    parameters TEXT,
                    environment_variables TEXT,
                    started_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    finished_at TIMESTAMP,
                    output TEXT,
                    error_message TEXT,
                    exit_code INTEGER,
                    FOREIGN KEY (program_name) REFERENCES program_instances (name)
                )
            """)
            
            # Create indexes for better performance
            cursor.execute("""
                CREATE INDEX IF NOT EXISTS idx_program_instances_template 
                ON program_instances(template_name)
            """)
            
            cursor.execute("""
                CREATE INDEX IF NOT EXISTS idx_program_runs_program 
                ON program_runs(program_name)
            """)
            
            cursor.execute("""
                CREATE INDEX IF NOT EXISTS idx_program_runs_status 
                ON program_runs(status)
            """)
            
            conn.commit()
    
    @contextmanager
    def get_connection(self):
        """Context manager for database connections"""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row  # Enable column access by name
        try:
            yield conn
        except Exception as e:
            conn.rollback()
            raise e
        finally:
            conn.close()
    
    def get_database_stats(self) -> Dict[str, int]:
        """Get database statistics"""
        try:
            with self.get_connection() as conn:
                cursor = conn.cursor()
                
                cursor.execute("SELECT COUNT(*) as count FROM program_templates")
                template_count = cursor.fetchone()['count']
                
                cursor.execute("SELECT COUNT(*) as count FROM program_instances")
                instance_count = cursor.fetchone()['count']
                
                cursor.execute("SELECT COUNT(*) as count FROM program_runs")
                runs_count = cursor.fetchone()['count']
                
                return {
                    'template_count': template_count,
                    'instance_count': instance_count,
                    'runs_count': runs_count
                }
        except Exception as e:
            print(f"Error getting database stats: {e}")
            return {'template_count': 0, 'instance_count': 0, 'runs_count': 0}
    
    def backup_database(self, backup_path: str) -> bool:
        """Create a backup of the database"""
        try:
            import shutil
            shutil.copy2(self.db_path, backup_path)
            return True
        except Exception as e:
            print(f"Error creating database backup: {e}")
            return False
