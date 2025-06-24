"""
Program template store module for managing program templates in the database.
"""

import json
from typing import Dict, List, Optional, Any
from .models import ProgramTemplate
from .base import DatabaseConnection
from ..interfaces import ProgramTemplateStoreInterface


class ProgramTemplateStore(ProgramTemplateStoreInterface):
    """Store for program templates"""
    
    def __init__(self, db_connection: DatabaseConnection):
        self.db_connection = db_connection
    
    def save(self, template: ProgramTemplate) -> bool:
        """Save or update a program template in the database"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    INSERT OR REPLACE INTO program_templates 
                    (name, model_class_name, schema, updated_at)
                    VALUES (?, ?, ?, CURRENT_TIMESTAMP)
                """, (
                    template.name,
                    template.model_class.__name__,
                    json.dumps(template.schema)
                ))
                conn.commit()
                return True
        except Exception as e:
            print(f"Error saving program template {template.name}: {e}")
            return False
    
    def get(self, name: str) -> Optional[Dict[str, Any]]:
        """Get a program template by name"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT name, model_class_name, schema, created_at, updated_at
                    FROM program_templates 
                    WHERE name = ?
                """, (name,))
                row = cursor.fetchone()
                
                if row:
                    return {
                        'name': row['name'],
                        'model_class_name': row['model_class_name'],
                        'schema': json.loads(row['schema']),
                        'created_at': row['created_at'],
                        'updated_at': row['updated_at']
                    }
                return None
        except Exception as e:
            print(f"Error getting program template {name}: {e}")
            return None
    
    def get_all(self) -> List[Dict[str, Any]]:
        """Get all program templates"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                cursor.execute("""
                    SELECT name, model_class_name, schema, created_at, updated_at
                    FROM program_templates
                    ORDER BY name
                """)
                rows = cursor.fetchall()
                
                return [
                    {
                        'name': row['name'],
                        'model_class_name': row['model_class_name'],
                        'schema': json.loads(row['schema']),
                        'created_at': row['created_at'],
                        'updated_at': row['updated_at']
                    }
                    for row in rows
                ]
        except Exception as e:
            print(f"Error getting all program templates: {e}")
            return []
    
    def delete(self, name: str) -> bool:
        """Delete a program template"""
        try:
            with self.db_connection.get_connection() as conn:
                cursor = conn.cursor()
                # First check if there are any instances using this template
                cursor.execute("""
                    SELECT COUNT(*) as count FROM program_instances 
                    WHERE template_name = ?
                """, (name,))
                count = cursor.fetchone()['count']
                
                if count > 0:
                    print(f"Cannot delete template {name}: {count} instances are using it")
                    return False
                
                cursor.execute("DELETE FROM program_templates WHERE name = ?", (name,))
                conn.commit()
                return cursor.rowcount > 0
        except Exception as e:
            print(f"Error deleting program template {name}: {e}")
            return False
