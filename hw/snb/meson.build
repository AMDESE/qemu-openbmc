# SPDX-License-Identifier: GPL-2.0-or-later

system_ss.add(when: 'CONFIG_SNB', if_true: files(
  'snb-channel.c',
  'snb-server.c',
  'snb-transport.c',
))
system_ss.add(when: 'CONFIG_SNB_I2C_BRIDGE', if_true: files('snb-i2c-bridge.c'))
