#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import struct
import socket


RADIO169_PORT = 31340

r169_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
r169_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
r169_sock.bind(('0.0.0.0', RADIO169_PORT))





def parsePayload(payload):
    assert len(payload) == 8

    # Typically payload[0] is 0x80
    if payload[0] != 0x80:
        print(" ************* BAD PAYLOAD")
        return

    # special buttons:
    btn_start = bool(payload[1] & 0x10)
    btn_back  = bool(payload[1] & 0x20)
    btn_logi  = bool(payload[1] & 0x40)
    btn_wtf   = bool(payload[1] & 0x80)  # unused, I think

    # The four buttons on the front-side:
    btn_LB    = bool(payload[1] & 0x01)
    btn_LT    = bool(payload[1] & 0x02)
    btn_RB    = bool(payload[1] & 0x04)
    btn_RT    = bool(payload[1] & 0x08)

    # The four individual buttons on the right:
    btn_Y     = bool(payload[2] & 0x10)  # orange/yellow
    btn_A     = bool(payload[2] & 0x20)  # green
    btn_B     = bool(payload[2] & 0x40)  # red
    btn_X     = bool(payload[2] & 0x80)  # blue

    # The 4-way button switchpad on the left:
    btn_up    = bool(payload[2] & 0x01)
    btn_down  = bool(payload[2] & 0x02)
    btn_right = bool(payload[2] & 0x04)
    btn_left  = bool(payload[2] & 0x08)

    # The 2 analog joysticks:
    L_lr, L_ud, R_lr, R_ud = struct.unpack('bbbb', payload[3:7])

    leftjoy_lr = L_lr-64
    leftjoy_ud = L_ud-64
    rightjoy_lr = R_lr-64
    rightjoy_ud = R_ud-64

    # payload[8] seems to be undefined/reserved

    specials = ''
    if btn_start:
        specials += 'start '
    if btn_back:
        specials += 'back '
    if btn_logi:
        specials += 'logicool '
    if btn_wtf:
        specials += 'wtf '

    fronts = ''
    if btn_LB:
        fronts += 'LB '
    if btn_LT:
        fronts += 'LT '
    if btn_RB:
        fronts += 'RB '
    if btn_RT:
        fronts += 'RT '

    pad = ''
    if btn_up:
        pad += 'up '
    if btn_down:
        pad += 'down '
    if btn_left:
        pad += 'left '
    if btn_right:
        pad += 'right '


    colors = ''
    if btn_Y:
        colors += 'orange '
    if btn_A:
        colors += 'green '
    if btn_B:
        colors += 'red '
    if btn_X:
        colors += 'blue '

    print(leftjoy_lr, leftjoy_ud, rightjoy_lr, rightjoy_ud, specials, fronts, pad, colors)






while True:
    pkt, addr = r169_sock.recvfrom(64)

    print('pkt len:', len(pkt), pkt.hex())
    if len(pkt) == 8:
        parsePayload(pkt)




