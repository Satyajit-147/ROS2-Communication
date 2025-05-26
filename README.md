# ROS 2 Communication - Week 2

This document summarizes the core communication concepts in **ROS 2**, with a focus on its decentralized architecture and the technologies that enable it.

## Why ROS2?

ROS1 lacks important requirements such as:
- Real-time communication (uses a central master node and TCP-based communication)
- Safety (no built-in support for secure communication)
- Security (no encryption and authentication of nodes and there is no way to communcate if the master node breaks down)
- Certification
- Compatibility (libraries and packages may not be the same in all versions of ROS1)

## ROS2 vs ROS1 Communication:

### ROS 1:
- A central server node, **Master node** is required to ensure communcation between nodes
- Communcation protocols: TCP ROS and UDP ROS

#### TCP ROS (Transmission Control Protocol)
TCP ROS protocol works by establishing a connection between sender and reciever (Handshaking). Once connection is established, the protocol breaks down the message into packets called segments and assigns a number to each packet to represent the order in which the message needs to be delivered.

Key Features of TCP:
Key Features of TCP:

| Feature             | Description                                                                                       |
|---------------------|---------------------------------------------------------------------------------------------------|
| Connection-Oriented | Requires connection setup before data transfer                                                    |
| Reliable            | Guarantees data delivery without loss                                                             |
| Ordered Delivery    | Data arrives in the same order as sent                                                            |
| Error Checking      | Detects and retransmits lost or corrupted packets                                                 |
| Flow Control        | Controls data flow (doesn't transmit when the network is busy)                                    |
| Congestion Control  | Adjusts transmission rate based on network congestion                                             |
| Stream-Oriented     | Data is transmitted as a continuous stream of bytes                                               |


#### UDP ROS (User Datagram Protocol)
UDP ROS is a connectionless protocol that works by sending data packets called datagrams without establishing a prior connection (No Handshaking). Each datagram includes the source and destination address but no sequence numbers.

Key Features of UDP:
| Feature           | Description                                                                                              |
|------------------|----------------------------------------------------------------------------------------------------------|
| Connectionless    | No setup required; packets sent independently                                                            |
| Unreliable        | No guarantee of packet delivery, order, or duplication avoidance                                         |
| No Congestion     | Sender can flood network, possibly causing packet loss                                                   |
| No Flow Control   | No mechanism to slow down sender if receiver is overwhelmed                                              |
| Low Overhead      | Minimal header and processing overhead (only 8-bits long and contains essential information like length  |
|                   | of datagram, source and destination port)                                                                |
| Message-Oriented  | Each packet is independent, treated as a separate message                                                |


### ROS2
ROS 2 uses a decentralized system instead of a central Master Node. It relies on the DDS (Data Distribution Service) protocol, which allows nodes to automatically discover each other and communicate directly in a peer-to-peer manner.

#### Problems with Master Node
- If it fails, the system breaks
- Hard to scale for many nodes
- Needs manual setup for networking
- No security built-in
- Doesn’t work well with changing networks
- Slows things down by handling all discovery

#### DDS (Data Dsitribution Service)
DDS is a protocol thst enables real-time/high-performance data exchange between publishers and subscribers

#### How DDS Works in ROS 2
Publishers and Subscribers are created like in ROS 1.

DDS handles:
- Finding other nodes (discovery)
- Managing data flow (communication)
- Handling network issues, retries, etc.
- Communication is peer-to-peer, meaning each node sends data directly to interested nodes (subscribers), without a central hub.

#### Peer-to-Peer Communication in ROS2
Peer-to-peer (P2P) communication means that each node (peer) in the ROS 2 network communicates directly with other nodes without needing to route messages through a central server or broker.

- Nodes communicate **directly with each other** without a central server.
- DDS handles **discovery** of nodes and their topics automatically.
- Once discovered, nodes establish **direct network connections** (between the publisher and the subscriber).
- Publishers send data **straight to matching subscribers**.
- Communication uses **UDP or TCP** (to transmit the message), managed by DDS.
- **Quality of Service (QoS)** settings control reliability and performance.
- No single point of failure — if one node fails, others keep working.
- Direct links reduce **latency** and improve speed.
- System can **scale easily** by adding more nodes without bottlenecks.
- Nodes can **join or leave anytime** without disrupting others.
- DDS optimizes connections for **real-time performance**.


#### QoS (Quality of Service):
This enables the user to customize the behaviour of message delivery between subscriber and the publisher. It allows the user to add more detail to the reliability, delivery timing, and data lifespan of the message being delivered. A publisher can only send the message to the subscriber if their QoS profiles match (compatible). 

Some QoS Policies:
| **QoS Policy** | **Options** | **Explanation** |
|----------------|-------------|------------------|
| **Reliability** | `RELIABLE`, `BEST_EFFORT` | `RELIABLE` ensures all messages are received (with retries). `BEST_EFFORT` delivers messages if possible (no retries). |
| **Durability** | `VOLATILE`, `TRANSIENT_LOCAL` | `TRANSIENT_LOCAL` stores the last message so late-joining subscribers can receive it. `VOLATILE` does not. |
| **History** | `KEEP_LAST`, `KEEP_ALL` | `KEEP_LAST(n)` keeps only the last `n` samples. `KEEP_ALL` stores every message (needs more memory). |
| **Deadline** | Time duration | Specifies how often a message is expected. DDS checks if messages come within the deadline. |
| **Liveliness** | `AUTOMATIC`, `MANUAL_BY_TOPIC` | Determines how DDS knows a node is alive. Useful for fault detection. |
| **Lifespan** | Time duration | Limits how long a message is valid. After that, it’s discarded even if not delivered. |






