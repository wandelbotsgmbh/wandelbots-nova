"""
Program Run Processor Implementations

This module contains different implementations of program run processors
that showcase different execution strategies (AsyncIO, Threading, Multiprocessing).
"""

import asyncio
import threading
import sys
import json
import time
import traceback
import tempfile
import os
from typing import Dict, Any, Callable

from .interface import ProgramRunProcessorInterface


class AsyncioProgramRunProcessor(ProgramRunProcessorInterface):
    """Program run processor using AsyncIO for concurrent execution"""
    
    async def execute_program(
        self,
        program_name: str,
        run_id: str,
        program_function: Callable,
        program_model_instance: Any,
        template_data: Dict[str, Any],
        instance_data: Dict[str, Any],
        parameters: Dict[str, Any],
        environment_variables: Dict[str, str]
    ) -> Dict[str, Any]:
        """Execute program using AsyncIO"""
        start_time = time.time()
        
        try:
            print(f"[AsyncIO Processor] Starting execution of {program_name} (run_id: {run_id})")
            
            # If the function is async, await it directly
            if asyncio.iscoroutinefunction(program_function):
                result = await program_function(program_model_instance)
            else:
                # Run sync function in thread pool to avoid blocking
                loop = asyncio.get_event_loop()
                result = await loop.run_in_executor(None, program_function, program_model_instance)
            
            execution_time = time.time() - start_time
            
            print(f"[AsyncIO Processor] Successfully executed {program_name} in {execution_time:.2f}s")
            return {
                'status': 'success',
                'result': result,
                'error': None,
                'execution_time': execution_time,
                'processor_type': 'asyncio',
                'metadata': {
                    'is_coroutine': asyncio.iscoroutinefunction(program_function),
                    'run_in_executor': not asyncio.iscoroutinefunction(program_function)
                }
            }
            
        except Exception as e:
            execution_time = time.time() - start_time
            error_msg = f"Error in AsyncIO processor: {str(e)}"
            print(f"[AsyncIO Processor] Error executing {program_name}: {error_msg}")
            
            return {
                'status': 'error',
                'result': None,
                'error': error_msg,
                'execution_time': execution_time,
                'processor_type': 'asyncio',
                'metadata': {
                    'exception_type': type(e).__name__,
                    'traceback': traceback.format_exc()
                }
            }


class ThreadProgramRunProcessor(ProgramRunProcessorInterface):
    """Program run processor using threading for parallel execution"""
    
    async def execute_program(
        self,
        program_name: str,
        run_id: str,
        program_function: Callable,
        program_model_instance: Any,
        template_data: Dict[str, Any],
        instance_data: Dict[str, Any],
        parameters: Dict[str, Any],
        environment_variables: Dict[str, str]
    ) -> Dict[str, Any]:
        """Execute program using threading"""
        start_time = time.time()
        
        try:
            print(f"[Thread Processor] Starting execution of {program_name} (run_id: {run_id})")
            
            result_container = {}
            exception_container = {}
            
            def thread_worker():
                try:
                    # Set environment variables for this thread
                    original_env = {}
                    for key, value in environment_variables.items():
                        original_env[key] = os.environ.get(key)
                        os.environ[key] = value
                    
                    try:
                        # Execute the program function
                        if asyncio.iscoroutinefunction(program_function):
                            # Handle async functions in thread
                            loop = asyncio.new_event_loop()
                            asyncio.set_event_loop(loop)
                            try:
                                result = loop.run_until_complete(program_function(program_model_instance))
                            finally:
                                loop.close()
                        else:
                            result = program_function(program_model_instance)
                        
                        result_container['result'] = result
                        
                    finally:
                        # Restore original environment variables
                        for key, original_value in original_env.items():
                            if original_value is None:
                                os.environ.pop(key, None)
                            else:
                                os.environ[key] = original_value
                                
                except Exception as e:
                    exception_container['exception'] = e
                    exception_container['traceback'] = traceback.format_exc()
            
            # Create and start thread
            thread = threading.Thread(target=thread_worker, name=f"nova-{program_name}-{run_id}")
            thread.start()
            
            # Wait for thread to complete (with a reasonable timeout)
            thread.join(timeout=300)  # 5 minutes timeout
            
            execution_time = time.time() - start_time
            
            if thread.is_alive():
                # Thread is still running, this means timeout
                error_msg = "Thread execution timeout after 300 seconds"
                print(f"[Thread Processor] Timeout executing {program_name}: {error_msg}")
                return {
                    'status': 'error',
                    'result': None,
                    'error': error_msg,
                    'execution_time': execution_time,
                    'processor_type': 'thread',
                    'metadata': {
                        'timeout': True,
                        'thread_name': thread.name
                    }
                }
            
            if 'exception' in exception_container:
                # Exception occurred in thread
                error_msg = f"Error in thread processor: {str(exception_container['exception'])}"
                print(f"[Thread Processor] Error executing {program_name}: {error_msg}")
                return {
                    'status': 'error',
                    'result': None,
                    'error': error_msg,
                    'execution_time': execution_time,
                    'processor_type': 'thread',
                    'metadata': {
                        'exception_type': type(exception_container['exception']).__name__,
                        'traceback': exception_container['traceback'],
                        'thread_name': thread.name
                    }
                }
            
            # Success case
            result = result_container.get('result')
            print(f"[Thread Processor] Successfully executed {program_name} in {execution_time:.2f}s")
            return {
                'status': 'success',
                'result': result,
                'error': None,
                'execution_time': execution_time,
                'processor_type': 'thread',
                'metadata': {
                    'thread_name': thread.name,
                    'is_coroutine': asyncio.iscoroutinefunction(program_function)
                }
            }
            
        except Exception as e:
            execution_time = time.time() - start_time
            error_msg = f"Error in thread processor setup: {str(e)}"
            print(f"[Thread Processor] Setup error for {program_name}: {error_msg}")
            
            return {
                'status': 'error',
                'result': None,
                'error': error_msg,
                'execution_time': execution_time,
                'processor_type': 'thread',
                'metadata': {
                    'exception_type': type(e).__name__,
                    'traceback': traceback.format_exc(),
                    'setup_error': True
                }
            }


class ProcessProgramRunProcessor(ProgramRunProcessorInterface):
    """Program run processor using multiprocessing for isolated execution"""
    
    async def execute_program(
        self,
        program_name: str,
        run_id: str,
        program_function: Callable,
        program_model_instance: Any,
        template_data: Dict[str, Any],
        instance_data: Dict[str, Any],
        parameters: Dict[str, Any],
        environment_variables: Dict[str, str]
    ) -> Dict[str, Any]:
        """Execute program using multiprocessing"""
        import pickle
        
        start_time = time.time()
        
        try:
            print(f"[Process Processor] Starting execution of {program_name} (run_id: {run_id})")
            
            # Create a temporary file to store the execution script
            with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as temp_file:
                temp_script_path = temp_file.name
                
                # Write the execution script
                script_content = f'''
import sys
import json
import pickle
import traceback
import os

def main():
    try:
        # Set environment variables
        env_vars = {json.dumps(environment_variables)}
        for key, value in env_vars.items():
            os.environ[key] = value
        
        # Load the serialized data
        with open('{temp_script_path}.data', 'rb') as f:
            program_function, program_model_instance = pickle.load(f)
        
        # Execute the program
        result = program_function(program_model_instance)
        
        # Save result
        with open('{temp_script_path}.result', 'wb') as f:
            pickle.dump({{'status': 'success', 'result': result}}, f)
            
    except Exception as e:
        # Save error
        with open('{temp_script_path}.result', 'wb') as f:
            pickle.dump({{
                'status': 'error',
                'error': str(e),
                'exception_type': type(e).__name__,
                'traceback': traceback.format_exc()
            }}, f)

if __name__ == '__main__':
    main()
'''
                temp_file.write(script_content)
            
            # Serialize the function and model instance
            data_file = temp_script_path + '.data'
            with open(data_file, 'wb') as f:
                pickle.dump((program_function, program_model_instance), f)
            
            # Execute the script in a subprocess
            import subprocess
            
            process = subprocess.Popen(
                [sys.executable, temp_script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env={**os.environ, **environment_variables}
            )
            
            # Wait for process to complete (with timeout)
            try:
                stdout, stderr = process.communicate(timeout=300)  # 5 minutes timeout
                execution_time = time.time() - start_time
                
                # Load result
                result_file = temp_script_path + '.result'
                if os.path.exists(result_file):
                    with open(result_file, 'rb') as f:
                        result_data = pickle.load(f)
                    
                    if result_data['status'] == 'success':
                        print(f"[Process Processor] Successfully executed {program_name} in {execution_time:.2f}s")
                        return {
                            'status': 'success',
                            'result': result_data['result'],
                            'error': None,
                            'execution_time': execution_time,
                            'processor_type': 'process',
                            'metadata': {
                                'pid': process.pid,
                                'return_code': process.returncode,
                                'stdout': stdout.decode() if stdout else '',
                                'stderr': stderr.decode() if stderr else ''
                            }
                        }
                    else:
                        error_msg = f"Error in process: {result_data['error']}"
                        print(f"[Process Processor] Error executing {program_name}: {error_msg}")
                        return {
                            'status': 'error',
                            'result': None,
                            'error': error_msg,
                            'execution_time': execution_time,
                            'processor_type': 'process',
                            'metadata': {
                                'pid': process.pid,
                                'return_code': process.returncode,
                                'exception_type': result_data.get('exception_type'),
                                'traceback': result_data.get('traceback'),
                                'stdout': stdout.decode() if stdout else '',
                                'stderr': stderr.decode() if stderr else ''
                            }
                        }
                else:
                    error_msg = "Process completed but no result file found"
                    return {
                        'status': 'error',
                        'result': None,
                        'error': error_msg,
                        'execution_time': execution_time,
                        'processor_type': 'process',
                        'metadata': {
                            'pid': process.pid,
                            'return_code': process.returncode,
                            'no_result_file': True
                        }
                    }
                    
            except subprocess.TimeoutExpired:
                process.kill()
                execution_time = time.time() - start_time
                error_msg = "Process execution timeout after 300 seconds"
                print(f"[Process Processor] Timeout executing {program_name}: {error_msg}")
                return {
                    'status': 'error',
                    'result': None,
                    'error': error_msg,
                    'execution_time': execution_time,
                    'processor_type': 'process',
                    'metadata': {
                        'pid': process.pid,
                        'timeout': True
                    }
                }
            
        except Exception as e:
            execution_time = time.time() - start_time
            error_msg = f"Error in process processor setup: {str(e)}"
            print(f"[Process Processor] Setup error for {program_name}: {error_msg}")
            
            return {
                'status': 'error',
                'result': None,
                'error': error_msg,
                'execution_time': execution_time,
                'processor_type': 'process',
                'metadata': {
                    'exception_type': type(e).__name__,
                    'traceback': traceback.format_exc(),
                    'setup_error': True
                }
            }
            
        finally:
            # Cleanup temporary files
            try:
                if 'temp_script_path' in locals():
                    for cleanup_file in [temp_script_path, temp_script_path + '.data', temp_script_path + '.result']:
                        if os.path.exists(cleanup_file):
                            os.unlink(cleanup_file)
            except Exception:
                pass  # Ignore cleanup errors
