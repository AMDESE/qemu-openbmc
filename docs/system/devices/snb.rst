.. SPDX-License-Identifier: GPL-2.0-or-later

Simulator network bridge (SNB)
==============================

The simulator network bridge connects QEMU device models to an external
simulator, called the peer, so that one machine can be split between the
two. For example, a BMC can run in QEMU while the host it manages runs in
the peer: the eSPI controller, the I2C and I3C targets and the board
signals of the host stay in the peer, and SNB carries the traffic between
them and the BMC's own devices.

QEMU devices
------------

``snb-transport``
  Holds the connection to the peer: two stream sockets, either TCP
  (``ip``, ``master-port``, default 5300, and ``slave-port``, default 5301)
  or Unix (``unix-master-path`` and ``unix-slave-path``). QEMU connects
  once the machine is created and reconnects whenever the peer goes away,
  so it runs whether or not the peer is up.

  When QEMU waits for the peer longer than ``stall-hold-ms`` (default 100),
  the guest clock is held until the answer comes, so that a slow peer does
  not look like a device timeout to the guest; 0 turns this off. The device
  blocks migration.

The bridge devices below each link to a transport with ``transport`` and
take an ``endpoint-id``, unique on the transport, that names the device on
the wire. The default ids are 2 for I2C, 4 for I3C, 6 for GPIO and 7 for
eSPI. With ``bus-name`` set, a bridge also tells the peer the name of the
bus it serves each time the transport connects, for a peer that has
several.

``snb-i2c-bus-bridge``
  An I2C device that answers on the bus it sits on for targets the peer
  holds: every unicast address, or those listed in ``addresses``, separated
  by colons. A read fetches ``read-size`` bytes (default 256) from the peer
  as it starts. The peer can drive the ``alert`` GPIO output by setting the
  signal ``ALERT_L``.

``snb-i3c-bus-bridge``
  Puts the I3C and legacy I2C targets the peer holds on the bus of the
  I3C controller named by ``ctrl``. The bridge creates them when the
  transport first connects; the peer's in-band interrupts are raised
  through the controller.

``snb-gpio-bridge``
  Carries named signals between the peer and the LTPI GPIO controller at
  ``ltpi-gpio-path``. ``line-names`` names the controller's lines,
  separated by semicolons: the input of pair n at offset 2n, its output at
  2n + 1. A signal the peer sets drives its input; a change on a named
  output goes to the peer, and so does the level of every named output
  after reset and on each connection. A signal listed in ``active-low``, or
  with ``polarity-from-suffix`` (the default) named with an ``_L`` suffix,
  is asserted low.

``snb-espi-bridge``
  Connects an eSPI controller in the peer to the AST2700 eSPI target at
  ``espi-path``: flash and peripheral cycles, configuration, virtual wires,
  alert and eSPI reset.

Example::

    qemu-system-aarch64 -M ast2700-evb ... \
        -device snb-transport,id=snb0,ip=127.0.0.1 \
        -device snb-espi-bridge,transport=snb0,espi-path=/machine/soc/espi[0] \
        -device snb-i3c-bus-bridge,transport=snb0,ctrl=/machine/soc/i3c-hci/ctrl[4] \
        -device snb-i2c-bus-bridge,transport=snb0,bus=aspeed.i2c.bus.1,addresses=0x3c:0x4c

Protocol
--------

All values are little-endian and all structures packed;
``include/hw/snb/snb-proto.h`` describes every message.

A packet is a 16-byte head tag, a 32-bit payload size, a 16-byte payload
header and the message. The head tag lets a receiver find the next packet
in a damaged stream. The payload header carries the message type, a
request id, flags and the endpoint id. The message type holds the bus
class in bits 27:24 and the operation in bits 23:0; bit 31 marks a
response, which echoes the request's id and endpoint and sets bit 0 of the
flags.

QEMU sends its requests on the master socket, where the peer answers
them; the peer sends its requests on the slave socket, where QEMU
answers them.
Some messages, such as signal changes, are notifications that nothing
answers.

A session starts on the slave socket, then the master socket: on each, both
sides send HELLO with a major and minor version, and a different major
version, or an older minor version than the oldest the other side
supports, ends the attempt. Each bridge with a ``bus-name`` then sends
BUS_REGISTER. When either socket goes away the session ends, and QEMU
starts a new one.
