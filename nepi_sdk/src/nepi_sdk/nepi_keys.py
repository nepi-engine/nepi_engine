#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#


# NEPI application-layer password hashing for named credentials.
# Uses PBKDF2-HMAC-SHA256 with a per-entry random salt.
# Credentials are stored as one-way hashes in rui_key — not reversible.


import hashlib
import os
import secrets
import yaml

from nepi_sdk import nepi_sdk
log_name = "nepi_keys"
logger = nepi_sdk.logger(log_name=log_name)

PBKDF2_ITERATIONS = 260000

RUI_KEY_PATH = '/opt/nepi/nepi_rui/src/rui_webserver/rui-app/src/keys/rui_key'


def create_rui_key():
    success = False
    key_path = os.path.dirname(RUI_KEY_PATH)
    if os.path.exists(key_path) == False:
        os.path.mkdir(key_path)
    if os.path.exists(key_path):
        # rui_key = Create key process
        # success = Save rui_key to RUI_KEY_PATH
        pass
    return success


def decrypt_rui_msg(msg):
    dmsg = msg
    if os.path.exists(RUI_KEY_PATH):
        # dmsg = Decrypt msg process
        pass
    return dmsg

def encrypt_rui_msg(msg):
    emsg = msg
    if os.path.exists(RUI_KEY_PATH):
        # emsg = Encrypt msg process
        pass
    return emsg