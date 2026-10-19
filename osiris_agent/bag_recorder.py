"""Bag recording: runs `ros2 bag record`, lists finished bags and uploads one to the gateway."""

import asyncio
import http.client
import json
import os
import signal
import subprocess
import tempfile
import time
import urllib.parse
import zipfile

from .topic_names import are_valid_topic_names

# What makes a folder a ROS 2 bag: ros2 bag record writes it when a recording
# ends, for every storage format.
BAG_METADATA = 'metadata.yaml'


def bag_folder(bag_dir, path):
    """The bag folder path names, if a listing may show it and a download may
    read it: a folder, not a symlink, directly in bag_dir, holding a finished
    ROS 2 bag. Returns (its absolute path, None), or (None, why not).

    bag_output_dir is set on the robot only, but this is the boundary of what
    a request from Osiris may read on the robot, so nothing outside a bag
    gets past it."""
    if not isinstance(path, str) or not path:
        return None, 'no path'
    bag_dir = os.path.abspath(bag_dir)
    folder = os.path.abspath(path)
    if os.path.dirname(folder) != bag_dir:
        return None, 'not directly in bag_output_dir'
    if os.path.islink(folder):
        return None, 'a symlink'
    if not os.path.isdir(folder):
        return None, 'not a folder'
    metadata = os.path.join(folder, BAG_METADATA)
    if os.path.islink(metadata) or not os.path.isfile(metadata):
        return None, f'not a ROS 2 bag (no {BAG_METADATA})'
    return folder, None


def bag_files(folder):
    """The bag's own files, for its zip and its size: regular files in the
    folder and its subfolders, never through a symlink."""
    for root, dirs, files in os.walk(folder, followlinks=False):
        dirs[:] = sorted(d for d in dirs if not os.path.islink(os.path.join(root, d)))
        for name in sorted(files):
            file = os.path.join(root, name)
            if not os.path.islink(file) and os.path.isfile(file):
                yield file


class BagRecorderMixin:

    def _bag_output_dir(self) -> str:
        return os.path.abspath(os.path.expanduser(
            self.get_parameter('bag_output_dir').get_parameter_value().string_value
        ))

    def _get_bag_files_snapshot(self) -> list[dict]:
        """Return metadata for every finished bag in the output directory."""
        bag_dir = self._bag_output_dir()
        bags = []
        try:
            names = sorted(entry.name for entry in os.scandir(bag_dir))
        except OSError:
            return bags
        for name in names:
            folder, _ = bag_folder(bag_dir, os.path.join(bag_dir, name))
            if folder is None:
                continue
            # Skip the directory that is currently being recorded.
            with self._bag_lock:
                if folder == self._bag_output_path:
                    continue
            try:
                size_bytes = sum(os.path.getsize(f) for f in bag_files(folder))
            except OSError:
                size_bytes = 0
            # Prefer the timestamp embedded in the directory name (bag_<ts>),
            # fall back to the directory mtime.
            try:
                created_at = float(name.split('_', 1)[1])
            except (IndexError, ValueError):
                try:
                    created_at = os.stat(folder).st_mtime
                except OSError:
                    created_at = None
            bags.append({
                'name':       name,
                'path':       folder,
                'size_bytes': size_bytes,
                'created_at': created_at,
            })
        return bags

    async def _send_bag_files(self):
        bags = await asyncio.to_thread(self._get_bag_files_snapshot)
        await self._send_queue.put(json.dumps({
            'type':      'bag_files',
            'bags':      bags,
            'timestamp': time.time(),
        }))
        self.get_logger().info(f"[bag] bag_files sent to gateway  count={len(bags)}")

    def _send_bag_download(self, path: str, request_id: str, upload_url: str):
        """Zip the bag directory and POST it to the gateway upload endpoint."""

        # Only a finished bag directly in bag_output_dir, and only its own files.
        folder, why_not = bag_folder(self._bag_output_dir(), path)
        if folder is None:
            self.get_logger().error(f"[bag] download refused: {path} is {why_not}")
            return

        tmp_path = None
        try:
            name = os.path.basename(folder)
            self.get_logger().info(
                f"[bag] zipping {folder}  request_id={request_id}"
            )

            with tempfile.NamedTemporaryFile(suffix='.zip', delete=False) as tmp_f:
                tmp_path = tmp_f.name

            with zipfile.ZipFile(tmp_path, 'w', zipfile.ZIP_DEFLATED) as zf:
                for file in bag_files(folder):
                    zf.write(file, os.path.relpath(file, os.path.dirname(folder)))

            zip_size = os.path.getsize(tmp_path)
            self.get_logger().info(
                f"[bag] zip ready  name={name}.zip  size={zip_size} bytes  "
                f"uploading to {upload_url}"
            )

            # ── Stream-upload via stdlib http.client (no extra dependencies) ───
            # Derive host/scheme from the active WS URL so the upload reaches
            # the same endpoint even when the gateway sends 'localhost' (which
            # would resolve to the container itself, not the host).
            ws_parsed     = urllib.parse.urlparse(self.ws_url)
            http_scheme   = 'https' if ws_parsed.scheme == 'wss' else 'http'
            netloc        = ws_parsed.netloc.split('?')[0]  # strip any query fragment
            upload_path   = urllib.parse.urlparse(upload_url).path
            qs            = urllib.parse.urlencode({'request_id': request_id})
            path_q        = f"{upload_path}?{qs}"

            self.get_logger().info(
                f"[bag] effective upload target: {http_scheme}://{netloc}{path_q}"
            )

            conn = (
                http.client.HTTPSConnection(netloc, timeout=120)
                if http_scheme == 'https'
                else http.client.HTTPConnection(netloc, timeout=120)
            )
            try:
                with open(tmp_path, 'rb') as f:
                    conn.request(
                        'POST', path_q, body=f,
                        headers={
                            'Content-Type':   'application/zip',
                            'Content-Length': str(zip_size),
                        },
                    )
                resp = conn.getresponse()
                resp.read()  # drain so the connection can be reused / closed cleanly
                self.get_logger().info(
                    f"[bag] upload complete  status={resp.status}  "
                    f"request_id={request_id}"
                )
            finally:
                conn.close()

        except Exception as e:
            self.get_logger().error(
                f"[bag] download upload failed  request_id={request_id}: {e}"
            )
        finally:
            if tmp_path and os.path.exists(tmp_path):
                try:
                    os.unlink(tmp_path)
                    self.get_logger().debug(f"[bag] temp zip deleted: {tmp_path}")
                except OSError:
                    pass

    def _start_bag_recording(self, data: dict):
        with self._bag_lock:
            if self._bag_proc is not None and self._bag_proc.poll() is None:
                self.get_logger().warning(
                    "[bag] rejected start — recording already in progress"
                )
                self._enqueue({
                    'type': 'error',
                    'message': 'Bag recording already in progress',
                })
                return

            topics = data.get('topics') or []
            if not are_valid_topic_names(topics):
                self.get_logger().warning(
                    "[bag] rejected start — request contains an invalid topic name"
                )
                self._enqueue({
                    'type': 'error',
                    'message': 'Bag recording request contains an invalid topic name',
                })
                return

            bag_dir = self._bag_output_dir()
            os.makedirs(bag_dir, exist_ok=True)
            output_path = os.path.join(bag_dir, f'bag_{int(time.time())}')

            if topics:
                cmd = ['ros2', 'bag', 'record', '-o', output_path] + topics
            else:
                cmd = ['ros2', 'bag', 'record', '-a', '-o', output_path]

            self.get_logger().info(
                f"[bag] launching subprocess: {' '.join(cmd)}"
            )
            self._bag_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            self._bag_output_path = output_path
            self.get_logger().info(
                f"[bag] subprocess started  pid={self._bag_proc.pid}  output={output_path}"
            )

        self._enqueue({
            'type':      'bag_record_started',
            'path':      output_path,
            'topics':    topics if topics else 'all',
            'timestamp': time.time(),
        })
        self.get_logger().info(
            f"[bag] bag_record_started sent to gateway  topics={topics or 'all'}"
        )

    def _stop_bag_recording(self):
        with self._bag_lock:
            if self._bag_proc is None or self._bag_proc.poll() is not None:
                self.get_logger().warning(
                    "[bag] rejected stop — no recording is currently in progress"
                )
                self._enqueue({
                    'type':    'error',
                    'message': 'No bag recording is currently in progress',
                })
                return

            proc = self._bag_proc
            output_path = self._bag_output_path
            self._bag_proc = None
            self._bag_output_path = None

        # SIGINT lets ros2 bag flush the SQLite index before exiting.
        self.get_logger().info(
            f"[bag] sending SIGINT to pid={proc.pid}  output={output_path}"
        )
        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=10)
            self.get_logger().info("[bag] subprocess exited cleanly")
        except subprocess.TimeoutExpired:
            self.get_logger().warning(
                "[bag] subprocess did not exit within 10 s — sending SIGKILL"
            )
            proc.kill()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass

        size_bytes = 0
        try:
            size_bytes = sum(os.path.getsize(f) for f in bag_files(output_path))
        except (FileNotFoundError, OSError):
            pass

        name = os.path.basename(output_path)
        self.get_logger().info(
            f"[bag] recording finalised  name={name}  size={size_bytes} bytes"
        )
        self._enqueue({
            'type':       'bag_record_stopped',
            'name':       name,
            'path':       output_path,
            'size_bytes': size_bytes,
            'timestamp':  time.time(),
        })
        self.get_logger().info(
            f"[bag] bag_record_stopped sent to gateway  size={size_bytes} bytes"
        )
