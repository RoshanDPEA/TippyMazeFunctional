# dpea-p2p

A module implementing a peer-to-peer communication protocol over TCP/IP.

A server begins listening on a particular port, and a client then connects to it via its IP.

The two can then send "packets" containing a packet type, specified by a shared enum, and a payload.

## Example usage

On the server:

```python
import enum
from dpea_p2p import Server

class PacketType(enum.Enum):
    NULL = 0
    COMMAND1 = 1
    COMMAND2 = 2

#         |Bind IP       |Port |Packet enum
s = Server("172.17.21.2", 5001, PacketType)
s.open_server()
s.wait_for_connection()

assert s.recv_packet() == (PacketType.COMMAND1, b"Hello!")
s.send_packet(PacketType.COMMAND2, b"Hello back!")

s.close_connection()
s.close_server()
```

On the client:

```python
import enum
from dpea_p2p import Client

class PacketType(enum.Enum):
    NULL = 0
    COMMAND1 = 1
    COMMAND2 = 2

#         |Server IP     |Port |Packet enum
c = Client("172.17.21.2", 5001, PacketType)
c.connect()

c.send_packet(PacketType.COMMAND1, b"Hello!")
assert c.recv_packet() == (PacketType.COMMAND2, b"Hello back!")

c.close_connection()
```

## Packet Structure

The packet structure looks like this:

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                          Packet Type                          |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                         Payload Length                        |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                                                               |
+                                                               +
|                                                               |
+                            Payload                            +
|                                                               |
+                                                               +
|                                                               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```
