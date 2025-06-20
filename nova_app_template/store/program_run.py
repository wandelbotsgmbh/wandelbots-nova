"""
Program run store module for managing program runs in the database.
"""

import json
from typing import Dict, List, Optional, Any
from .base import DatabaseConnection
from ..interfaces import ProgramRunStoreInterface


class ProgramRunStore(ProgramRunStoreInterface):
    """Store for program runs"""
    
    def __init__(self, db_connection: DatabaseConnection):
        self.db_connection = db_connection
    
    def save(self, run_data: Dict[str, Any]) -> bool:
        """Save a new program run in the database"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    INSERT INTO program_runs 
                    (run_id, program_name, status, parameters, environment_variables, started_at)
                    VALUES (?, ?, ?, ?, ?, ?)
                """, (
                    run_data['run_id'],
                    run_data['program_name'],
                    run_data['status'],
                    json.dumps(run_data.get('parameters', {})),
                    json.dumps(run_data.get('environment_variables', {})),
                    run_data['started_at']
                ))
                conn.commit()
                return True
        except Exception as e:
            print(f"Error saving program run {run_data['run_id']}: {e}")
            return False
    
    def get(self, run_id: str) -> Optional[Dict[str, Any]]:
        """Get a program run by run_id"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT run_id, program_name, status, parameters, environment_variables,
                           started_at, finished_at, output, error_message, exit_code
                    FROM program_runs 
                    WHERE run_id = ?
                """, (run_id,))
                row = cursor.fetchone()
                
                if row:
                    return {
                        'run_id': row['run_id'],
                        'program_name': row['program_name'],
                        'status': row['status'],
                        'parameters': json.loads(row['parameters'] or '{}'),
                        'environment_variables': json.loads(row['environment_variables'] or '{}'),
                        'started_at': row['started_at'],
                        'finished_at': row['finished_at'],
                        'output': row['output'],
                        'error_message': row['error_message'],
                        'exit_code': row['exit_code']
                    }
                return None
        except Exception as e:
            print(f"Error getting program run {run_id}: {e}")
            return None
    
    def get_by_program(self, program_name: str) -> List[Dict[str, Any]]:
        """Get all program runs for a specific program"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT run_id, program_name, status, parameters, environment_variables,
                           started_at, finished_at, output, error_message, exit_code
                    FROM program_runs 
                    WHERE program_name = ?
                    ORDER BY started_at DESC
                """, (program_name,))
                rows = cursor.fetchall()
                
                return [
                    {
                        'run_id': row['run_id'],
                        'program_name': row['program_name'],
                        'status': row['status'],
                        'parameters': json.loads(row['parameters'] or '{}'),
                        'environment_variables': json.loads(row['environment_variables'] or '{}'),
                        'started_at': row['started_at'],
                        'finished_at': row['finished_at'],
                        'output': row['output'],
                        'error_message': row['error_message'],
                        'exit_code': row['exit_code']
                    }
                    for row in rows
                ]
        except Exception as e:
            print(f"Error getting program runs for {program_name}: {e}")
            return []
    
    def update_status(self, run_id: str, status: str, **kwargs) -> bool:
        """Update the status and completion details of a program run"""
        try:
            # Extract kwargs for backwards compatibility
            finished_at = kwargs.get('finished_at')
            output = kwargs.get('output')
            error_message = kwargs.get('error_message')
            exit_code = kwargs.get('exit_code')
            
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    UPDATE program_runs 
                    SET status = ?, finished_at = ?, output = ?, error_message = ?, exit_code = ?
                    WHERE run_id = ?
                """, (status, finished_at, output, error_message, exit_code, run_id))
                conn.commit()
                return cursor.rowcount > 0
        except Exception as e:
            print(f"Error updating program run {run_id}: {e}")
            return False
    
    def delete(self, run_id: str) -> bool:
        """Delete a program run"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("DELETE FROM program_runs WHERE run_id = ?", (run_id,))
                conn.commit()
                return cursor.rowcount > 0
        except Exception as e:
            print(f"Error deleting program run {run_id}: {e}")
            return False
    
    def get_all(self) -> List[Dict[str, Any]]:
        """Get all program runs"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT run_id, program_name, status, parameters, environment_variables,
                           started_at, finished_at, output, error_message, exit_code
                    FROM program_runs 
                    ORDER BY started_at DESC
                """)
                rows = cursor.fetchall()
                
                return [
                    {
                        'run_id': row['run_id'],
                        'program_name': row['program_name'],
                        'status': row['status'],
                        'parameters': json.loads(row['parameters'] or '{}'),
                        'environment_variables': json.loads(row['environment_variables'] or '{}'),
                        'started_at': row['started_at'],
                        'finished_at': row['finished_at'],
                        'output': row['output'],
                        'error_message': row['error_message'],
                        'exit_code': row['exit_code']
                    }
                    for row in rows
                ]
        except Exception as e:
            print(f"Error getting all program runs: {e}")
            return []
