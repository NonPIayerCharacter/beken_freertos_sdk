
:link_to_translation:`zh_CN:[中文]`

Wi-Fi packet capture instructions
==================================================
Excellent network analyzer - OmniPeek
-----------------------------------------------------
Introduction to Omnipeek
+++++++++++++++++++++++++++++++++++++++++++++++++++
Known as the world's most powerful network protocol analyzer, OmniPeek was originally a product of Savvius (formerly known as WildPackets), the world's leading packet capture and analysis company. Savvius has been a leader in packet capture, deep packet inspection and network diagnostics solutions for 25 years and has now been acquired by Liveaction. Liveaction previously focused on network data flow level analysis. After acquiring Savvius, the two integrated and complemented each other to achieve a comprehensive analysis platform from flows to data packets. OmniPeek can analyze data packets Carry out in-depth analysis, analyze thousands of protocols, comprehensively analyze network traffic from multiple dimensions, provide dozens of intuitive chart reports, and provide expert-level analysis and diagnosis of network events.

OmniPeek panel
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. image:: ../../_static/omni_panel.png

OmniPeek provides multiple dimensions of analysis methods and charts for use. The picture above shows the compass panel, which is an interactive forensic dashboard that displays network utilization over time, including events, protocols, flows, nodes, Channel, VLAN, data rate, application and country statistics. These statistics are displayed in the optional data source widget and can be viewed from a live capture or from a single capture file.

OmniPeek application analysis
+++++++++++++++++++++++++++++++++++++++++++++++++++
The Applications dashboard displays key statistics of applications in the capture window. This application visibility provides insight into user behavior and traffic patterns on the network at certain times of day, week, month, or year. It can help analysts better understand who is going to what web sites and using which applications when.

OmniPeek statistics analysis
+++++++++++++++++++++++++++++++++++++++++++++++++++
.. image:: ../../_static/omni_graphs.png

OmniPeek and tthe Capture Engines calculate a variety of key statistics in real time and present these statistics in intuitive graphical displays. You can save, copy, print, or automatically generate periodic reports on these statistics in a variety of formats.

Summarize
+++++++++++++++++++++++++++++++++++++++++++++++++++
OmniPeek is a powerful network analyzer that can be easily used by beginners. However, because its functions are too rich, the above brief introduction cannot give everyone a thorough understanding. For more details, please visit the official website. `Onmipeek official website <https://www.liveaction.com/>`_

OmniPeek drivers
---------------------------------------------------
According to the driver information in the software installation directory ``Omnipeek\Drivers``, OmniPeek currently supports two types of packet capture network cards. One is based on the Atheros chip ``Atheros USB 802.11n Wireless LAN card``, which only supports 11N mode; the other is based on ``Ralink RT2870``, which can support 802.11b/g/n/ac. It is recommended to use the two USB packet capture network cards ``ASUS USB-AC55`` or ``Netgear A6210`` .

Packet capture scenarios
---------------------------------------------------
- Wi-Fi connection issues, analyze key frames through packet capture.
- Wi-Fi performance issues, analyze retransmission, aggregation degree, PHY rate, etc. through packet capture.
- Wi-Fi power consumption issues, analyze PM value through packet capture, etc.

Common issues
---------------------------------------------------
OmniPeek packet capture examples
+++++++++++++++++++++++++++++++++++++++++++++++++++
- Scanning phase

.. image:: ../../_static/scan.png

- Authentication phase

.. image:: ../../_static/auth.png

- Four-way handshake phase

.. image:: ../../_static/eapol.png

- Decryption

If you can capture the above complete authentication process, you can enter the SSID and password to decrypt through ``Tools-Decrypt WLAN Packets``.

.. image:: ../../_static/decrypt.png


