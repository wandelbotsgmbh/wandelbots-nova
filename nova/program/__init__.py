from nova.program.function import ProgramPreconditions, program
from nova.program.runner import ProgramRunner
from nova.program.nats_key_value_store import KeyValueStore
from nova.program.function import Program

# ProgramStore = KeyValueStore[Program] would be better but python doesn't support this
# when I do store = ProgramStore() the __orig_class__ is not available in the __init__
# my reseach say's python doesn't capture the type argument when I do this

# TODO: change the Program with wandelbots_api_client.v2.models.Program 
class ProgramStore(KeyValueStore[Program]):
    def __init__(self, nats_bucket_name, nats_client_config = None, nats_kv_config = None):
        super().__init__(Program, nats_bucket_name, nats_client_config, nats_kv_config)


__all__ = ["ProgramRunner", "program", "ProgramPreconditions", "ProgramStore"]
