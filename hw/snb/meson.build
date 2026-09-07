# SPDX-License-Identifier: GPL-2.0-or-later

system_ss.add(when: 'CONFIG_SNB', if_true: files(
  'snb-channel.c',
  'snb-server.c',
  'snb-transport.c',
))
system_ss.add(when: 'CONFIG_SNB_I2C_BRIDGE', if_true: files('snb-i2c-bridge.c'))
system_ss.add(when: 'CONFIG_SNB_I3C_BRIDGE', if_true: files(
  'snb-i3c-bridge.c',
  'snb-i3c-target.c',
))
system_ss.add(when: 'CONFIG_SNB_GPIO_BRIDGE', if_true: files('snb-gpio-bridge.c'))
system_ss.add(when: 'CONFIG_SNB_ESPI_BRIDGE', if_true: files('snb-espi-bridge.c'))
