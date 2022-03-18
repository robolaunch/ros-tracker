from pydantic import BaseModel

class MemoryUsageOut(BaseModel):
    used: int
    total: int

class NetworkUsageOut(BaseModel):
    kbytes_sent: float
    kbytes_recv: float



class SystemOut(BaseModel):
    memory_usage: MemoryUsageOut
    cpu_usage: float
    cpu_core_usage: list
    network_usage: NetworkUsageOut
    uptime: float

class GPUOut(BaseModel):
    gpu_id: int
    gpu_name: str
    gpu_load: float
    gpu_free_memory: int
    gpu_used_memory: int
    gpu_total_memory: int
    gpu_temperature: float
    gpu_uuid: str



