# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""
Example code for using the payload service API
"""
import argparse
import io
import logging
import struct
import sys
import time

import bosdyn.api.payload_pb2 as payload_protos
import bosdyn.api.robot_id_pb2 as robot_id_protos
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.payload import PayloadClient
from bosdyn.client.payload_registration import (PayloadRegistrationClient,
                                                PayloadRegistrationKeepAlive)

LOGGER = logging.getLogger()


def payload_spot(config):
    """A simple example of using the Boston Dynamics API to communicate payload configs with spot.

    First registers a payload then lists all payloads on robot, including newly registered payload.
    """

    sdk = bosdyn.client.create_standard_sdk('PayloadSpotClient')

    robot = sdk.create_robot(config.hostname)

    # Authenticate robot before being able to use it
    bosdyn.client.util.authenticate(robot)

    # Create a payload registration client
    payload_registration_client = robot.ensure_client(
        PayloadRegistrationClient.default_service_name)

    # Create a payload
    payload = payload_protos.Payload()
    payload.GUID, payload_secret = bosdyn.client.util.read_or_create_payload_credentials(
        config.payload_credentials_file_1)
    payload.name = 'Client Registered Payload Ex #1'
    payload.description = 'This payload was created and registered by the payloads.py client example.'
    payload.label_prefix.append('test_payload')
    payload.is_authorized = False
    payload.is_enabled = False
    payload.is_noncompute_payload = False
    payload.version.major_version = 1
    payload.version.minor_version = 1
    payload.version.patch_level = 1
    # note: this field is not required, but highly recommended
    payload.mount_frame_name = payload_protos.MOUNT_FRAME_BODY_PAYLOAD

    # Register the payload
    payload_registration_client.register_payload(payload, payload_secret)

    # Create a payload client
    payload_client = robot.ensure_client(PayloadClient.default_service_name)

    # Update the payload version
    version = robot_id_protos.SoftwareVersion()
    version.major_version = 2
    version.minor_version = 2
    version.patch_level = 2
    payload_registration_client.update_payload_version(payload.GUID, payload_secret, version)

    # List all payloads
    payloads = payload_client.list_payloads()
    print('\n\nPayload Listing\n' + '-' * 40)
    print(payloads)

    # Use the PayloadRegistrationKeepAlive to register a payload

    # Create a payload
    payload = payload_protos.Payload()
    payload.GUID, payload_secret = bosdyn.client.util.read_or_create_payload_credentials(
        config.payload_credentials_file_2)
    payload.name = 'Client Registered Payload Ex #2'
    payload.description = 'This payload was created and registered by the payloads.py using a PayloadRegistrationKeepAlive.'
    payload.label_prefix.append('test_payload')
    payload.is_authorized = False
    payload.is_enabled = False
    payload.is_noncompute_payload = False
    payload.version.major_version = 3
    payload.version.minor_version = 2
    payload.version.patch_level = 1
    # note: this field is not required, but highly recommended
    payload.mount_frame_name = payload_protos.MOUNT_FRAME_BODY_PAYLOAD

    # Create and start the keep alive
    keep_alive = PayloadRegistrationKeepAlive(payload_registration_client, payload, payload_secret)
    keep_alive.start()

    # List all payloads
    payloads = payload_client.list_payloads()
    print('\n\nPayload Listing\n' + '-' * 40)
    print(payloads)

    print('\n\n\nDon\'t forget to clean up these registrations in the Spot payloads webpage!')


def main():
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument(
        '--payload-credentials-file-1', required=True, type=str, help=
        'Filename on this script\'s host where the GUID and secret for the first example payload are generated (per robot) and stored.'
    )
    parser.add_argument(
        '--payload-credentials-file-2', required=True, type=str, help=
        'Filename on this script\'s host where the GUID and secret for the second example payload are generated (per robot) and stored.'
    )
    options = parser.parse_args()

    payload_spot(options)

    return True


if __name__ == '__main__':
    if not main():
        sys.exit(1)
