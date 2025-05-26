# ROS 2 Communication Overview

This document summarizes the core communication concepts in **ROS 2**, with a focus on its decentralized architecture and the technologies that enable it.

## Why ROS2?

ROS1 lacks most important requirements such as:
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
| Feature           | Description                                               |
|-------------------|-----------------------------------------------------------|
| Connection-Oriented | Requires connection setup before data transfer             |
| Reliable          | Guarantees data delivery without loss                      |
| Ordered Delivery  | Data arrives in the same order as sent                      |
| Error Checking    | Detects and retransmits lost or corrupted packets          |
| Flow Control      | Controls data flow (doesn't transmit when the network is busy)         |
| Congestion Control| Adjusts transmission rate based on network congestion      |
| Stream-Oriented   | Data is transmitted as a continuous stream of bytes        |

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


In ROS 2:
- The system is **fully decentralized**.
- It uses the **Data Distribution Service (DDS)** middleware to support robust, scalable, and flexible communication.

This shift improves:
- **Reliability** (no single point of failure)
- **Scalability** (more devices, more nodes)
- **Real-time performance**
- **Support for embedded and distributed systems**

---

## 🌐 DDS (Data Distribution Service)

- DDS is a **middleware protocol** used in ROS 2 to handle message exchange between nodes.
- Based on the **publish-subscribe** model.
- Manages **automatic discovery**, **data serialization**, **transport protocols**, **QoS settings**, and more.

**Advantages of DDS:**
- Decentralized communication
- Built-in discovery
- Multiple transport layers (TCP, UDP)
- Quality of Service (QoS) control (latency, reliability, deadline, etc.)

---

## 📡 Communication Protocols

### 1. **UDP (User Datagram Protocol)**
- DDS often uses **UDP** for faster, connectionless message passing.
- Suitable for real-time or lossy environments (e.g., robotics, drones).
- Supports **multicast**, helping in efficient data distribution to multiple subscribers.

### 2. **TCP (Transmission Control Protocol)**
- Also supported via DDS when reliability is prioritized over speed.
- Ensures message delivery but adds latency.

DDS can **dynamically select** the best transport protocol depending on system needs and QoS settings.

---

## 🤝 Peer-to-Peer Discovery

- ROS 2 nodes **discover each other automatically** without needing a central master.
- Every node broadcasts its presence over the network using **Simple Discovery Protocol (SDP)** provided by DDS.
- This enables **true peer-to-peer communication**.

### Benefits:
- Resilience to failure
- Flexible in dynamic environments
- Suitable for distributed and multi-robot systems

---

## 🗺️ Discovery Server (Optional for ROS 2)

While peer-to-peer discovery works well, large networks can suffer from discovery overhead.

**Discovery Server**:
- An optional DDS feature used to improve performance in large or complex networks.
- Acts as a lightweight coordinator that stores and shares node information.
- Used in cases like:
  - Multi-robot systems across WANs
  - Reducing discovery traffic
  - Improving startup time in massive deployments

---

## ✅ Summary

| Feature                  | ROS 1                          | ROS 2 (with DDS)                    |
|--------------------------|--------------------------------|-------------------------------------|
| Central Master           | Required                       | ❌ Not required                     |
| Communication            | Custom TCP (ROS TCPROS)        | DDS (UDP, TCP, Multicast)           |
| Discovery                | Centralized (via master)       | Peer-to-peer (via DDS)              |
| Scalability              | Limited                        | High (supports distributed systems) |
| Fault Tolerance          | Low (master failure = halt)    | High (no single point of failure)   |
| QoS Control              | ❌ No                          | ✅ Yes (latency, reliability, etc.)  |

---

## 📚 References
- ROS 2 Documentation: https://docs.ros.org/en/rolling/index.html
- DDS Overview by OMG: https://www.omg.org/spec/DDS/
- Fast DDS Discovery Server: https://fast-dds.docs.eprosima.com/en/latest/

---

