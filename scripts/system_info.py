import psutil
import time

_boot_time = psutil.boot_time()

# This function returns the memory used in the system in kilobytes
def getMemoryUsage():
    # get the memory usage
    memory_usage = psutil.virtual_memory()
    # return the dictionary
    return {'used': memory_usage.used/1024, 'total': memory_usage.total/1024}

# Returns all processes and their CPU usages in a list
# This function takes too much time, because while taking CPU usages, we get an average usage of "0.01" seconds for each process.
# This must be executed in a separate thread.
def getProcessCpuUsage():
    process_cpu_usage = []
    for proc in psutil.process_iter():
        try:
            # Get process name & pid from process object.
            processName = proc.name()
            processID = proc.pid

            process_cpu_usage.append([processName, processID, proc.cpu_percent(interval=0.01)])
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return process_cpu_usage

# This function returns the total CPU usage of the system
def getTotalCpuUsage():
    # this returns 0 in every first call, I don't know the reason
    total_cpu_usage = psutil.cpu_percent()
    core_cpu_usage = psutil.cpu_percent(percpu=True)
    return total_cpu_usage, core_cpu_usage

# This function returns the network usage of the system as a dictionary in kilobytes
# This can be used to get the error rate or drop rate in the network in future!
def getNetworkUsageDict():
    # get the network usage
    network_usage = psutil.net_io_counters()
    # return the dictionary
    return {'kbytes_sent': network_usage.bytes_sent/1024, 'kbytes_recv': network_usage.bytes_recv/1024}

def getUptime():
    return time.time() - _boot_time