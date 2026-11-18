.. SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
.. SPDX-License-Identifier: Apache-2.0

Zephyr integration
##################

This repository provides Puya PY32 device headers and HAL/LL drivers for
applications using Zephyr. The module name is ``hal_py32``.

The ``zephyr/module.yml`` descriptor delegates CMake and Kconfig integration
to ``modules/hal_py32`` in the Zephyr tree or another ``MODULE_EXT_ROOT``.
A Zephyr checkout must provide integration for this repository's original
layout; integration for a repackaged HAL directory is not interchangeable.

The integration selects device definitions and sources from
``Libraries/PY32F0xx_HAL_Driver``, ``Libraries/PY32F0xx_LL_Driver`` and
``Libraries/CMSIS/Device/PY32F0xx``. CMSIS core headers come from Zephyr's
CMSIS module. Firmware examples, startup assembly and linker scripts are
not part of the Zephyr build. Zephyr controls startup and vector placement.

The HAL configuration header is available in ``User/py32f0xx_hal_conf.h``.
The vendor system source supports ``FORBID_VECT_TAB_MIGRATION`` to exclude
its application-specific vector-table migration code.

Maintainership and synchronization follow the upstream repository's
contribution process. Zephyr integration changes belong in the Zephyr tree;
vendor driver updates retain their upstream paths and license notices.
Pin an exact repository revision in the west manifest for reproducible builds.
