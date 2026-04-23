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
# Credentials are stored as one-way hashes in rui_keys.yaml — not reversible.


import hashlib
import os
import secrets
import yaml

from nepi_sdk import nepi_sdk
log_name = "nepi_keys"
logger = nepi_sdk.logger(log_name=log_name)

PBKDF2_ITERATIONS = 260000


def hash_password(password):
    """
    Hashes a plaintext password using PBKDF2-HMAC-SHA256 with a cryptographically
    random 32-byte salt.  Returns a dict with keys 'hash' and 'salt', both as
    lowercase hex strings.
    """
    salt_bytes = secrets.token_bytes(32)
    hash_bytes = hashlib.pbkdf2_hmac(
        'sha256',
        password.encode('utf-8'),
        salt_bytes,
        PBKDF2_ITERATIONS
    )
    return {
        'hash': hash_bytes.hex(),
        'salt': salt_bytes.hex()
    }


def verify_password(password, stored_hash, stored_salt):
    """
    Verifies a plaintext password against stored hex hash and salt values.
    Uses a constant-time comparison to prevent timing attacks.
    Returns True if the password matches, False otherwise.
    """
    try:
        salt_bytes = bytes.fromhex(stored_salt)
        check_bytes = hashlib.pbkdf2_hmac(
            'sha256',
            password.encode('utf-8'),
            salt_bytes,
            PBKDF2_ITERATIONS
        )
        return secrets.compare_digest(check_bytes.hex(), stored_hash)
    except Exception as e:
        logger.log_warn("verify_password failed: " + str(e))
        return False


def load_rui_keys(keys_file_path):
    """
    Reads the rui_keys.yaml file at keys_file_path and returns its contents as a dict.
    Returns an empty dict if the file does not exist or cannot be parsed.
    """
    if not os.path.exists(keys_file_path):
        return {}
    try:
        with open(keys_file_path, 'r') as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        return data if isinstance(data, dict) else {}
    except Exception as e:
        logger.log_warn("Failed to load rui_keys file " + keys_file_path + ": " + str(e))
        return {}


def save_rui_keys(keys_file_path, keys_dict):
    """
    Writes keys_dict to the rui_keys.yaml file at keys_file_path.
    Creates parent directories if they do not already exist.
    Returns True on success, False on failure.
    """
    try:
        parent = os.path.dirname(keys_file_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(keys_file_path, 'w') as f:
            yaml.dump(keys_dict, stream=f, default_flow_style=False, sort_keys=False)
        return True
    except Exception as e:
        logger.log_warn("Failed to save rui_keys file " + keys_file_path + ": " + str(e))
        return False


def set_rui_key(keys_file_path, key_name, password):
    """
    Hashes password and stores the result in rui_keys.yaml under key_name.
    Creates the file if it does not exist.  Overwrites any existing entry for key_name.
    Returns True on success, False on failure.
    """
    keys_dict = load_rui_keys(keys_file_path)
    keys_dict[key_name] = hash_password(password)
    return save_rui_keys(keys_file_path, keys_dict)


def get_rui_key(keys_file_path, key_name):
    """
    Returns the stored hash dict for key_name from rui_keys.yaml.
    Returns None if key_name is not present or the file does not exist.
    """
    keys_dict = load_rui_keys(keys_file_path)
    return keys_dict.get(key_name, None)


def verify_rui_key(keys_file_path, key_name, password):
    """
    Loads the stored entry for key_name and verifies password against it.
    Returns True if the password matches, False if no match or the entry is absent.
    """
    entry = get_rui_key(keys_file_path, key_name)
    if entry is None:
        logger.log_warn("verify_rui_key: key '" + key_name + "' not found in " + keys_file_path)
        return False
    stored_hash = entry.get('hash', '')
    stored_salt = entry.get('salt', '')
    if not stored_hash or not stored_salt:
        logger.log_warn("verify_rui_key: incomplete entry for key '" + key_name + "'")
        return False
    return verify_password(password, stored_hash, stored_salt)
