"""
Program instance store module for managing program instances in the database.
"""

import json
from typing import Dict, List, Optional, Any
from .models import ProgramInstance
from .base import DatabaseConnection
from ..interfaces import ProgramInstanceStoreInterface


class ProgramInstanceStore(ProgramInstanceStoreInterface):
    """Store for program instances"""
    
    def __init__(self, db_connection: DatabaseConnection):
        self.db_connection = db_connection
    
    def save(self, instance: ProgramInstance) -> bool:
        """Save or update a program instance in the database"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    INSERT OR REPLACE INTO program_instances 
                    (name, template_name, data, updated_at)
                    VALUES (?, ?, ?, CURRENT_TIMESTAMP)
                """, (
                    instance.name,
                    instance.template.name,
                    json.dumps(instance.data)
                ))
                conn.commit()
                return True
        except Exception as e:
            print(f"Error saving program instance {instance.name}: {e}")
            return False
    
    def get(self, name: str) -> Optional[Dict[str, Any]]:
        """Get a program instance by name"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT pi.name, pi.template_name, pi.data, 
                           pi.created_at, pi.updated_at,
                           pt.model_class_name, pt.schema
                    FROM program_instances pi
                    JOIN program_templates pt ON pi.template_name = pt.name
                    WHERE pi.name = ?
                """, (name,))
                row = cursor.fetchone()
                
                if row:
                    return {
                        'name': row['name'],
                        'template_name': row['template_name'],
                        'data': json.loads(row['data']),
                        'created_at': row['created_at'],
                        'updated_at': row['updated_at'],
                        'template_schema': json.loads(row['schema']),
                        'template_model_class_name': row['model_class_name']
                    }
                return None
        except Exception as e:
            print(f"Error getting program instance {name}: {e}")
            return None
    
    def get_all(self) -> List[Dict[str, Any]]:
        """Get all program instances"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT pi.name, pi.template_name, pi.data, 
                           pi.created_at, pi.updated_at,
                           pt.model_class_name, pt.schema
                    FROM program_instances pi
                    JOIN program_templates pt ON pi.template_name = pt.name
                    ORDER BY pi.name
                """)
                rows = cursor.fetchall()
                
                return [
                    {
                        'name': row['name'],
                        'template_name': row['template_name'],
                        'data': json.loads(row['data']),
                        'created_at': row['created_at'],
                        'updated_at': row['updated_at'],
                        'template_schema': json.loads(row['schema']),
                        'template_model_class_name': row['model_class_name']
                    }
                    for row in rows
                ]
        except Exception as e:
            print(f"Error getting all program instances: {e}")
            return []
    
    def get_by_template(self, template_name: str) -> List[Dict[str, Any]]:
        """Get all program instances for a specific template"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT pi.name, pi.template_name, pi.data, 
                           pi.created_at, pi.updated_at,
                           pt.model_class_name, pt.schema
                    FROM program_instances pi
                    JOIN program_templates pt ON pi.template_name = pt.name
                    WHERE pi.template_name = ?
                    ORDER BY pi.name
                """, (template_name,))
                rows = cursor.fetchall()
                
                return [
                    {
                        'name': row['name'],
                        'template_name': row['template_name'],
                        'data': json.loads(row['data']),
                        'created_at': row['created_at'],
                        'updated_at': row['updated_at'],
                        'template_schema': json.loads(row['schema']),
                        'template_model_class_name': row['model_class_name']
                    }
                    for row in rows
                ]
        except Exception as e:
            print(f"Error getting program instances for template {template_name}: {e}")
            return []
    
    def delete(self, name: str) -> bool:
        """Delete a program instance"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("DELETE FROM program_instances WHERE name = ?", (name,))
                conn.commit()
                return cursor.rowcount > 0
        except Exception as e:
            print(f"Error deleting program instance {name}: {e}")
            return False
    
    def update_data(self, name: str, data: Dict[str, Any]) -> bool:
        """Update only the data field of a program instance"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    UPDATE program_instances 
                    SET data = ?, updated_at = CURRENT_TIMESTAMP
                    WHERE name = ?
                """, (json.dumps(data), name))
                conn.commit()
                return cursor.rowcount > 0
        except Exception as e:
            print(f"Error updating program instance data {name}: {e}")
            return False
