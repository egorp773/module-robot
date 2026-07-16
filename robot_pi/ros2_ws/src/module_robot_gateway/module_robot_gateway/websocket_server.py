"""Asyncio WebSocket server isolated from the rclpy executor thread."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import contextlib
import threading
from typing import Callable, Dict, Iterable, Optional, Tuple
import uuid

from websockets.exceptions import ConnectionClosed
from websockets.legacy.server import WebSocketServerProtocol, serve


CommandCallback = Callable[[str, str], str]
DisconnectCallback = Callable[[str], None]
ConnectCallback = Callable[[str], None]
LogCallback = Callable[[str], None]


@dataclass
class _Client:
    identifier: str
    websocket: WebSocketServerProtocol
    telemetry_queue: "asyncio.Queue[Tuple[str, ...]]"
    send_lock: asyncio.Lock
    sender_task: Optional["asyncio.Task[None]"] = None
    dropped_snapshots: int = 0


class GatewayWebSocketServer:
    """A bounded multi-client text server with drop-oldest telemetry queues."""

    def __init__(
        self,
        *,
        bind_address: str,
        port: int,
        path: str,
        max_clients: int,
        max_message_bytes: int,
        queue_depth: int,
        on_command: CommandCallback,
        on_connect: ConnectCallback,
        on_disconnect: DisconnectCallback,
        log_info: LogCallback,
        log_warning: LogCallback,
    ) -> None:
        self._bind_address = bind_address
        self._port = port
        self._path = path
        self._max_clients = max_clients
        self._max_message_bytes = max_message_bytes
        self._queue_depth = queue_depth
        self._on_command = on_command
        self._on_connect = on_connect
        self._on_disconnect = on_disconnect
        self._log_info = log_info
        self._log_warning = log_warning

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._server = None
        self._clients: Dict[str, _Client] = {}
        self._thread: Optional[threading.Thread] = None
        self._started = threading.Event()
        self._startup_error: Optional[BaseException] = None
        self._stopping = threading.Event()
        self._broadcast_lock = threading.Lock()
        self._pending_snapshot: Optional[Tuple[str, ...]] = None
        self._broadcast_scheduled = False

    def start(self, timeout_s: float = 5.0) -> None:
        if self._thread is not None:
            raise RuntimeError("WebSocket server already started")
        self._thread = threading.Thread(
            target=self._run,
            name="module-robot-websocket",
            daemon=True,
        )
        self._thread.start()
        if not self._started.wait(timeout_s):
            raise RuntimeError("timed out starting WebSocket server")
        if self._startup_error is not None:
            raise RuntimeError(
                f"failed to start WebSocket server: {self._startup_error}"
            ) from self._startup_error

    def stop(self, timeout_s: float = 5.0) -> None:
        if self._thread is None:
            return
        self._stopping.set()
        loop = self._loop
        if loop is not None and loop.is_running():
            future = asyncio.run_coroutine_threadsafe(self._shutdown_async(), loop)
            try:
                future.result(timeout=timeout_s)
            except Exception as exc:  # Shutdown remains best-effort and bounded.
                self._log_warning(f"WebSocket shutdown warning: {exc}")
            loop.call_soon_threadsafe(loop.stop)
        self._thread.join(timeout=timeout_s)
        if self._thread.is_alive():
            self._log_warning("WebSocket thread did not stop within timeout")
        self._thread = None

    def broadcast(self, messages: Iterable[str]) -> None:
        """Queue one telemetry snapshot for every client without blocking ROS."""

        snapshot = tuple(messages)
        if not snapshot:
            return
        loop = self._loop
        if loop is None or not loop.is_running() or self._stopping.is_set():
            return
        # Coalesce at the thread boundary as well as in each client queue. This
        # prevents the asyncio callback queue itself from growing if a client or
        # route-validation callback temporarily stalls the event loop.
        with self._broadcast_lock:
            self._pending_snapshot = snapshot
            if self._broadcast_scheduled:
                return
            self._broadcast_scheduled = True
        try:
            loop.call_soon_threadsafe(self._flush_pending_broadcast)
        except RuntimeError:
            with self._broadcast_lock:
                self._pending_snapshot = None
                self._broadcast_scheduled = False

    @property
    def client_count(self) -> int:
        return len(self._clients)

    def _run(self) -> None:
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._start_async())
        except BaseException as exc:
            self._startup_error = exc
            self._started.set()
            loop.close()
            self._loop = None
            return
        self._started.set()
        try:
            loop.run_forever()
        finally:
            pending = asyncio.all_tasks(loop)
            for task in pending:
                task.cancel()
            if pending:
                loop.run_until_complete(
                    asyncio.gather(*pending, return_exceptions=True)
                )
            loop.run_until_complete(loop.shutdown_asyncgens())
            loop.close()
            self._loop = None

    async def _start_async(self) -> None:
        self._server = await serve(
            self._handle_client,
            self._bind_address,
            self._port,
            max_size=self._max_message_bytes,
            max_queue=max(1, self._queue_depth),
            ping_interval=10.0,
            ping_timeout=10.0,
            close_timeout=2.0,
            compression=None,
            reuse_address=True,
        )
        self._log_info(
            f"WebSocket gateway listening on ws://{self._bind_address}:"
            f"{self._port}{self._path}"
        )

    async def _shutdown_async(self) -> None:
        server = self._server
        if server is not None:
            server.close()
        clients = list(self._clients.values())
        if clients:
            await asyncio.gather(
                *(
                    client.websocket.close(code=1001, reason="gateway shutdown")
                    for client in clients
                ),
                return_exceptions=True,
            )
        if server is not None:
            await server.wait_closed()
        self._server = None

    async def _handle_client(
        self, websocket: WebSocketServerProtocol, request_path: str
    ) -> None:
        if request_path != self._path:
            await websocket.close(code=1008, reason="invalid WebSocket path")
            return
        if len(self._clients) >= self._max_clients:
            with contextlib.suppress(ConnectionClosed):
                await websocket.send("ERR,CONNECT,MAX_CLIENTS")
            await websocket.close(code=1013, reason="maximum clients connected")
            return

        identifier = uuid.uuid4().hex
        client = _Client(
            identifier=identifier,
            websocket=websocket,
            telemetry_queue=asyncio.Queue(maxsize=self._queue_depth),
            send_lock=asyncio.Lock(),
        )
        self._clients[identifier] = client
        client.sender_task = asyncio.create_task(
            self._telemetry_sender(client), name=f"telemetry-{identifier[:8]}"
        )
        peer = websocket.remote_address
        self._log_info(f"WebSocket client connected id={identifier[:8]} peer={peer}")
        self._on_connect(identifier)

        try:
            await self._send_direct(client, "OK,CONNECTED,GATEWAY_STAGE1")
            # Existing Flutter uses this legacy readiness token and then sends
            # periodic PING probes. It grants no ARM or motion authority.
            await self._send_direct(client, "STATE,CONNECTED")
            async for raw_message in websocket:
                if not isinstance(raw_message, str):
                    await self._send_direct(client, "ERR,COMMAND,TEXT_REQUIRED")
                    continue
                if len(raw_message.encode("utf-8")) > self._max_message_bytes:
                    await websocket.close(code=1009, reason="message too large")
                    break
                try:
                    response = self._on_command(identifier, raw_message)
                except Exception as exc:  # Never let one command kill the server.
                    self._log_warning(
                        f"Unhandled gateway command error for {identifier[:8]}: {exc}"
                    )
                    response = "ERR,COMMAND,INTERNAL"
                if response:
                    await self._send_direct(client, response)
        except (ConnectionClosed, asyncio.TimeoutError):
            pass
        finally:
            self._clients.pop(identifier, None)
            if client.sender_task is not None:
                client.sender_task.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await client.sender_task
            self._on_disconnect(identifier)
            self._log_info(
                f"WebSocket client disconnected id={identifier[:8]} "
                f"dropped_telemetry={client.dropped_snapshots}"
            )

    async def _send_direct(self, client: _Client, message: str) -> None:
        async with client.send_lock:
            await asyncio.wait_for(client.websocket.send(message), timeout=2.0)

    async def _telemetry_sender(self, client: _Client) -> None:
        try:
            while True:
                snapshot = await client.telemetry_queue.get()
                for message in snapshot:
                    await self._send_direct(client, message)
        except (ConnectionClosed, asyncio.TimeoutError):
            with contextlib.suppress(ConnectionClosed, asyncio.TimeoutError):
                await asyncio.wait_for(
                    client.websocket.close(code=1011, reason="client send timeout"),
                    timeout=2.0,
                )
            return

    def _enqueue_snapshot(self, snapshot: Tuple[str, ...]) -> None:
        for client in tuple(self._clients.values()):
            queue = client.telemetry_queue
            if queue.full():
                with contextlib.suppress(asyncio.QueueEmpty):
                    queue.get_nowait()
                client.dropped_snapshots += 1
            with contextlib.suppress(asyncio.QueueFull):
                queue.put_nowait(snapshot)

    def _flush_pending_broadcast(self) -> None:
        with self._broadcast_lock:
            snapshot = self._pending_snapshot
            self._pending_snapshot = None
            self._broadcast_scheduled = False
        if snapshot:
            self._enqueue_snapshot(snapshot)
