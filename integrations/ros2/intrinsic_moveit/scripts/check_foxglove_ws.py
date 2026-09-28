#!/usr/bin/env python3
import argparse
import asyncio
import json
import struct
import sys

import websockets

REQUIRED_TOPICS = {
    '/robot_description',
    '/robot_description_web',
    '/tf',
    '/tf_static',
    '/joint_states',
    '/demo/scene_markers',
    '/demo/grasp_candidates',
    '/demo/grasp_poses',
    '/demo/pregrasp_poses',
    '/demo/selected_grasp',
    '/demo/planned_tcp_path',
    '/demo/status',
}
ASSET_URI = 'package://robot_hardware_description/meshes/robotiq_hande/robotiq_hande_body.dae'


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('url', nargs='?', default='ws://localhost:8765')
    args = parser.parse_args()
    async with websockets.connect(
            args.url, subprotocols=['foxglove.sdk.v1'], max_size=None) as ws:
        server_info = json.loads(await ws.recv())
        capabilities = server_info.get('capabilities') or []
        print('serverInfo', server_info.get('op'), 'capabilities', capabilities)
        if 'assets' not in capabilities:
            raise SystemExit('FAIL capabilities missing assets')

        topics = {}
        deadline = asyncio.get_event_loop().time() + 20.0
        while asyncio.get_event_loop().time() < deadline and not REQUIRED_TOPICS.issubset(topics):
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
            except asyncio.TimeoutError:
                continue
            if isinstance(msg, str):
                payload = json.loads(msg)
                if payload.get('op') == 'advertise':
                    for channel in payload.get('channels', []):
                        topics[channel['topic']] = channel['id']
        missing = sorted(REQUIRED_TOPICS - set(topics))
        print(f'advertised {len(topics)} topics')
        if missing:
            raise SystemExit(f'FAIL missing topics: {missing}')

        await ws.send(json.dumps({'op': 'fetchAsset', 'uri': ASSET_URI, 'requestId': 7}))
        asset_status = None
        asset_bytes = 0
        while True:
            msg = await asyncio.wait_for(ws.recv(), timeout=30.0)
            if isinstance(msg, (bytes, bytearray)) and msg and msg[0] == 0x04:
                _req_id, asset_status = struct.unpack_from('<IB', msg, 1)
                (err_len,) = struct.unpack_from('<I', msg, 6)
                asset_bytes = len(msg) - (10 + err_len)
                break
        print(f'fetchAsset status={asset_status} bytes={asset_bytes}')
        if asset_status != 0 or asset_bytes < 1_000_000:
            raise SystemExit('FAIL fetchAsset')

        channel_id = topics['/joint_states']
        await ws.send(json.dumps({
            'op': 'subscribe',
            'subscriptions': [{'id': 1, 'channelId': channel_id}],
        }))
        got_message = False
        deadline = asyncio.get_event_loop().time() + 5.0
        while asyncio.get_event_loop().time() < deadline:
            remaining = deadline - asyncio.get_event_loop().time()
            msg = await asyncio.wait_for(ws.recv(), timeout=max(remaining, 0.1))
            if isinstance(msg, (bytes, bytearray)) and msg and msg[0] == 0x01:
                got_message = True
                break
        if not got_message:
            raise SystemExit('FAIL no joint_states binary message within 5s')
        print('PASS foxglove websocket')


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except SystemExit:
        raise
    except Exception as exc:
        print(f'FAIL {exc}', file=sys.stderr)
        raise SystemExit(1) from exc
