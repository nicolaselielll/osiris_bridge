"""Telemetry: periodic system samples (CPU, memory, disk, network, processes, battery) and host details."""

import os
import platform
import time

import psutil

from osiris_agent import __version__ as AGENT_VERSION

TELEMETRY_INTERVAL         = 1.0   # seconds between telemetry samples
MAX_TELEMETRY_PROCESSES    = 15    # cap on processes reported per telemetry sample
# The process list is by far the most expensive part of a telemetry sample —
# a full psutil.process_iter() pass over every process on the host, then a
# oneshot() deep read (cmdline/memory/username/threads) for the top N — for
# something that doesn't meaningfully change second to second the way
# cpu/ram/disk/net do. Computed and sent only once every this many ticks;
# cpu/ram/disk/net/battery stay on the plain 1Hz TELEMETRY_INTERVAL. The
# client is expected to hold onto the last processes list it received
# in between, not clear it.
TELEMETRY_PROCESS_EVERY_N_TICKS = 5
# Disk and network rates are sent in MB/s to this many decimals: 0.1 KiB/s,
# the precision the Telemetry pane shows them in. Two decimals rounded the
# few KiB/s a robot typically sends down to 0.
RATE_DECIMALS = 4


def _is_loopback(interface: str) -> bool:
    # lo on Linux, lo0 on other systems.
    return interface == 'lo' or (interface.startswith('lo') and interface[2:].isdigit())


def network_bytes() -> dict:
    """Bytes sent and received so far on each network interface, loopback
    left out: the ROS 2 traffic between the robot's own processes goes over
    loopback, never leaves the robot, and counts each byte as both sent and
    received."""
    return {
        name: (io.bytes_sent, io.bytes_recv)
        for name, io in psutil.net_io_counters(pernic=True).items()
        if not _is_loopback(name)
    }


def network_rates_mbps(before: dict, after: dict, dt: float) -> tuple:
    """Send and receive rates in MB/s between two network_bytes() readings dt
    seconds apart. Only interfaces in both readings count, and a counter that
    went back (an interface that was reset) counts as nothing, so an
    interface coming or going doesn't show as a burst of traffic."""
    sent = recv = 0
    for name, (sent_after, recv_after) in after.items():
        if name in before:
            sent_before, recv_before = before[name]
            sent += max(0, sent_after - sent_before)
            recv += max(0, recv_after - recv_before)
    return round(sent / dt / (1024 * 1024), RATE_DECIMALS), round(recv / dt / (1024 * 1024), RATE_DECIMALS)


class TelemetryMixin:

    def _collect_telemetry(self):
        if not self.ws or not self.loop:
            return
        if not self._telemetry_enabled:
            return
        self._telemetry_tick += 1
        include_processes = (self._telemetry_tick % TELEMETRY_PROCESS_EVERY_N_TICKS == 0)
        self._enqueue({
            'type': 'telemetry',
            'data': self._get_telemetry_snapshot(include_processes=include_processes),
            'timestamp': time.time(),
        })

    def _on_battery_state(self, msg) -> None:
        """Cache the latest BatteryState message for inclusion in telemetry snapshots
        and for the send_command battery-level precondition check."""
        try:
            self._last_battery_state = {
                'percent':  round(float(msg.percentage) * 100.0, 1) if msg.percentage == msg.percentage else None,  # NaN guard
                'voltage':  round(float(msg.voltage), 3)  if msg.voltage  == msg.voltage  else None,
                'current':  round(float(msg.current), 3)  if msg.current  == msg.current  else None,
                'status':   int(msg.power_supply_status),
                'present':  bool(msg.present),
            }
            self._last_battery_state_time = time.time()
        except Exception:
            pass

    def _get_telemetry_snapshot(self, include_processes: bool = True) -> dict:
        cpu_now = round(psutil.cpu_percent(interval=None), 1)

        vm = psutil.virtual_memory()
        ram_percent = vm.percent

        now = time.time()
        disk_usage      = psutil.disk_usage('/')
        disk_read_mbps  = 0.0
        disk_write_mbps = 0.0
        try:
            disk_io = psutil.disk_io_counters()
            if self._last_disk_io is not None and self._last_io_time is not None:
                dt = now - self._last_io_time
                if dt > 0:
                    disk_read_mbps  = round(max(0.0, (disk_io.read_bytes  - self._last_disk_io.read_bytes)  / dt / (1024 * 1024)), RATE_DECIMALS)
                    disk_write_mbps = round(max(0.0, (disk_io.write_bytes - self._last_disk_io.write_bytes) / dt / (1024 * 1024)), RATE_DECIMALS)
            self._last_disk_io = disk_io
        except Exception:
            pass

        net_tx_mbps = 0.0
        net_rx_mbps = 0.0
        try:
            net_io = network_bytes()
            if self._last_net_io is not None and self._last_io_time is not None:
                dt = now - self._last_io_time
                if dt > 0:
                    net_tx_mbps, net_rx_mbps = network_rates_mbps(self._last_net_io, net_io, dt)
            self._last_net_io = net_io
        except Exception:
            pass

        self._last_io_time = now

        cpu_c = None
        try:
            temps = psutil.sensors_temperatures()
            for key in ('coretemp', 'cpu-thermal', 'acpitz', 'k10temp', 'cpu_thermal'):
                entries = temps.get(key)
                if entries:
                    cpu_c = round(entries[0].current, 1)
                    break
        except Exception:
            pass

        # CPU frequency (GHz)
        cpu_freq = None
        try:
            freq = psutil.cpu_freq()
            if freq and freq.current:
                cpu_freq = round(freq.current / 1000.0, 2)
        except Exception:
            pass

        # Load averages (1, 5, 15 min)
        cpu_load = None
        try:
            load1, load5, load15 = os.getloadavg()
            cpu_load = {
                'load1':  round(load1, 1),
                'load5':  round(load5, 1),
                'load15': round(load15, 1),
            }
        except Exception:
            pass

        # Process list (top processes by CPU usage).
        # Two-phase fetch: cheap fields for all processes first, then only
        # pull the more expensive fields (cmdline, memory_info, username) for
        # the top N CPU consumers, to avoid per-second syscalls against every
        # process on the host and to avoid leaking the full host process list
        # (cmdline can contain secrets) over the wire.
        #
        # psutil.Process.cpu_percent() is normalized to a SINGLE core by
        # default (100% = one core fully saturated, so a process pegging 2 of
        # 4 cores reports ~200%) — a different convention than cpu_now above,
        # which psutil already normalizes to the whole system (0-100%). Divide
        # by logical core count here so both numbers in this same pane mean
        # the same thing: percent of total system CPU capacity.
        try:
            logical_cpus = psutil.cpu_count(logical=True) or 1
        except Exception:
            logical_cpus = 1

        # Skipped entirely (not just omitted from the result) on ticks that
        # don't need it — this is the expensive part TELEMETRY_PROCESS_EVERY_N_
        # TICKS exists to avoid paying every second, not just a smaller payload.
        processes = []
        if include_processes:
            try:
                candidates = []
                for proc in psutil.process_iter(['pid', 'name', 'cpu_percent']):
                    try:
                        info = proc.info
                        candidates.append((round((info['cpu_percent'] or 0.0) / logical_cpus, 1), proc))
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
                candidates.sort(key=lambda c: c[0], reverse=True)

                for cpu_percent, proc in candidates[:MAX_TELEMETRY_PROCESSES]:
                    try:
                        with proc.oneshot():
                            cmdline = proc.cmdline()
                            mem_info = proc.memory_info()
                            processes.append({
                                'pid':          proc.pid,
                                'name':         proc.name(),
                                'cmdline':      ' '.join(cmdline)[:256] if cmdline else '',
                                'num_threads':  proc.num_threads(),
                                'username':     proc.username(),
                                'memory_mb':    round(mem_info.rss / (1024 * 1024), 1) if mem_info else 0,
                                'cpu_percent':  cpu_percent,
                            })
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
            except Exception:
                pass

        snapshot = {
            'cpu': {
                'now':        cpu_now,
                'throttling': None,
                'temp':       cpu_c,
                'freq':       cpu_freq,
                'load':       cpu_load,
            },
            'ram': {
                'percent':  round(ram_percent, 1),
                'used_mb':  round(vm.used  / (1024 * 1024), 1),
                'total_mb': round(vm.total / (1024 * 1024), 1),
            },
            'disk': {
                'percent':    round(disk_usage.percent, 1),
                'used_gb':    round(disk_usage.used  / (1024 ** 3), 2),
                'total_gb':   round(disk_usage.total / (1024 ** 3), 2),
                'read_mbps':  disk_read_mbps,
                'write_mbps': disk_write_mbps,
            },
            'net': {
                'tx_mbps': net_tx_mbps,
                'rx_mbps': net_rx_mbps,
            },
            'battery':   self._last_battery_state,
        }
        # Present only on ticks that actually computed it — absent, not an
        # empty list, so the client can tell "no update this tick" apart from
        # "genuinely no processes" and hold onto whatever it already has
        # instead of blanking the UI for the 4 out of 5 ticks this is skipped.
        if include_processes:
            snapshot['processes'] = processes
        return snapshot

    def _get_cpu_model(self) -> str | None:
        try:
            if os.path.exists('/proc/cpuinfo'):
                with open('/proc/cpuinfo') as f:
                    for line in f:
                        if line.lower().startswith(('model name', 'hardware', 'processor')):
                            _, value = line.split(':', 1)
                            value = value.strip()
                            if value:
                                return value
        except Exception:
            pass

        cpu_model = platform.processor() or platform.machine()
        return cpu_model or None

    def _get_robot_model(self) -> str | None:
        for env_name in ('OSIRIS_ROBOT_MODEL', 'ROBOT_MODEL'):
            value = os.environ.get(env_name)
            if value:
                return value

        for path in ('/proc/device-tree/model', '/sys/firmware/devicetree/base/model'):
            try:
                if os.path.exists(path):
                    with open(path, 'rb') as f:
                        value = f.read().decode(errors='ignore').strip('\x00\n ')
                        if value:
                            return value
            except Exception:
                pass
        return None

    def _get_initial_state_meta(self, telemetry: dict | None = None) -> dict:
        ram_total_mb = None
        try:
            ram_total_mb = telemetry.get('ram', {}).get('total_mb') if telemetry else None
            if ram_total_mb is None:
                ram_total_mb = round(psutil.virtual_memory().total / (1024 * 1024), 1)
        except Exception:
            pass

        return {
            'agentVersion': AGENT_VERSION,
            # The agent's own node, so a robot's agent can be told apart from
            # the others when several share a ROS domain.
            'agent_node': self.get_fully_qualified_name(),
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'cpu_model': self._get_cpu_model(),
            'cpu_cores': psutil.cpu_count(logical=False),
            'cpu_threads': psutil.cpu_count(logical=True),
            'ram_total_mb': ram_total_mb,
            'arch': platform.machine() or None,
            'robot_model': self._get_robot_model(),
        }
