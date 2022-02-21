import psutil

# This function returns the memory used in the system in kilobytes
def get_memory_usage():
    # get the memory usage
    memory_usage = psutil.virtual_memory()
    # return the dictionary
    return {'used': memory_usage.used/1024, 'total': memory_usage.total/1024}

# Returns all processes and their CPU usages in a list
# This function takes too much time, because while taking CPU usages, we get an average usage of "0.01" seconds for each process.
# This must be executed in a separate thread.
def get_process_cpu_usage():
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
def get_total_cpu_usage():
    # this returns 0 in every first call, I don't know the reason
    total_cpu_usage = psutil.cpu_percent()
    return total_cpu_usage

# This function returns the network usage of the system as a dictionary in kilobytes
# This can be used to get the error rate or drop rate in the network in future!
def get_network_usage_dict():
    # get the network usage
    network_usage = psutil.net_io_counters()
    # return the dictionary
    return {'kbytes_sent': network_usage.bytes_sent/1024, 'kbytes_recv': network_usage.bytes_recv/1024}