##
## /* SPDX-License-Identifier: GPL-2.0-only OR BSD-3-Clause */
## /******************************************************************************
##  *
##  * This file is provided under a dual license.  When you use or
##  * distribute this software, you may choose to be licensed under
##  * version 2 of the GNU General Public License ("GPLv2 License")
##  * or BSD License.
##  *
##  * GPLv2 License
##  *
##  * Copyright(C) 2019 MediaTek Inc.
##  *
##  * This program is free software; you can redistribute it and/or modify
##  * it under the terms of version 2 of the GNU General Public License as
##  * published by the Free Software Foundation.
##  *
##  * This program is distributed in the hope that it will be useful, but
##  * WITHOUT ANY WARRANTY; without even the implied warranty of
##  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##  * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
##  *
##  * BSD LICENSE
##  *
##  * Copyright(C) 2019 MediaTek Inc.
##  * All rights reserved.
##  *
##  * Redistribution and use in source and binary forms, with or without
##  * modification, are permitted provided that the following conditions
##  * are met:
##  *
##  *  * Redistributions of source code must retain the above copyright
##  *    notice, this list of conditions and the following disclaimer.
##  *  * Redistributions in binary form must reproduce the above copyright
##  *    notice, this list of conditions and the following disclaimer in
##  *    the documentation and/or other materials provided with the
##  *    distribution.
##  *  * Neither the name of the copyright holder nor the names of its
##  *    contributors may be used to endorse or promote products derived
##  *    from this software without specific prior written permission.
##  *
##  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
##  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
##  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
##  * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
##  * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
##  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
##  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
##  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
##  * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
##  * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
##  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##  *
##  *****************************************************************************/
##
##

# excerpt from bionic/libc/fs_config_generator.py
_FIXUPS = {
    'media_drm': 'mediadrm',
    'media_ex': 'mediaex',
    'media_codec': 'mediacodec'
}

# the table is translate from system/core/libcutils/include/private/android_filesystem_config.h
android_ids = {
    0: "root", 1: "daemon", 2: "bin",
   1000: "system", 1001: "radio", 1002: "bluetooth",
   1003: "graphics", 1004: "input", 1005: "audio",
   1006: "camera", 1007: "log", 1008: "compass",
   1009: "mount", 1010: "wifi", 1011: "adb",
   1012: "install", 1013: "media", 1014: "dhcp",
   1015: "sdcard_rw", 1016: "vpn", 1017: "keystore",
   1018: "usb", 1019: "drm", 1020: "mdnsr",
   1021: "gps", 1022: "unused1", 1023: "media_rw",
   1024: "mtp", 1025: "unused2", 1026: "drmrpc",
   1027: "nfc", 1028: "sdcard_r", 1029: "clat",
   1030: "loop_radio", 1031: "media_drm", 1032: "package_info",
   1033: "sdcard_pics", 1034: "sdcard_av", 1035: "sdcard_all",
   1036: "logd", 1037: "shared_relro", 1038: "dbus",
   1039: "tlsdate", 1040: "media_ex", 1041: "audioserver",
   1042: "metrics_coll", 1043: "metricsd", 1044: "webserv",
   1045: "debuggerd", 1046: "media_codec", 1047: "cameraserver",
   1048: "firewall", 1049: "trunks", 1050: "nvram",
   1051: "dns", 1052: "dns_tether", 1053: "webview_zygote",
   1054: "vehicle_network", 1055: "media_audio", 1056: "media_video",
   1057: "media_image", 1058: "tombstoned", 1059: "media_obb",
   1060: "ese", 1061: "ota_update", 1062: "automotive_evs",
   1063: "lowpan", 1064: "hsm", 1065: "reserved_disk",
   1066: "statsd", 1067: "incidentd", 1068: "secure_element",
   2000: "shell", 2001: "cache", 2002: "diag",
   2900: "oem_reserved_start", 2999: "oem_reserved_end", 3001: "net_bt_admin",
   3002: "net_bt", 3003: "inet", 3004: "net_raw",
   3005: "net_admin", 3006: "net_bw_stats", 3007: "net_bw_acct",
   3009: "readproc", 3010: "wakelock", 3011: "uhid",
   5000: "oem_reserved_2_start", 5999: "oem_reserved_2_end", 9997: "everybody",
   9998: "misc", 9999: "nobody", 10000: "app",
   10000: "app_start", 19999: "app_end", 20000: "cache_gid_start",
   29999: "cache_gid_end", 30000: "ext_gid_start", 39999: "ext_gid_end",
   40000: "ext_cache_gid_start", 49999: "ext_cache_gid_end", 50000: "shared_gid_start",
   59999: "shared_gid_end", 65534: "overflowuid", 99000: "isolated_start",
   99999: "isolated_end", 100000: "user", 100000: "user_offset"
}

fout = {
    "dev.inc": ["/* DO NOT EDIT. It's generated from dev.txt */"],
    "ueventd.dev.rc": ["# add the rules to ueventd.rc"],
    "init.dev.rc": ["# add the symlinks to init.mkn.rc", "on post-fs"],
}

def name2id(login):
    if login in android_ids:
        aid = android_ids[login]
        return _FIXUPS.get(aid, aid)
    else:
        return login

def parse_dev_txt():
        with open("dev.txt", "r") as f:
            for line in f:
                if line[0] == 'd':
                    dev, typ, mode, uid, gid, major, minor = (line+" 0 0").split()[:7]
                    uid, gid = int(uid), int(gid)
                    major, minor = int(major), int(minor)
                    if dev[0] != '/':
                        dev = '/'+dev

                    # dev.inc
                    name = dev.split('/')[-1]
                    if typ=="c":
                        devname = "/dev/"+name
                    else:
                        devname = "/dev/block/"+name
                    if typ in 'bc':
                        nodestr = '\t{ "%c%s", MKDEV(%d, %d) },' % (
                                typ, name, major, minor)
                        fout["dev.inc"].append(nodestr)

                    # ueventd.rc
                    if typ in 'bc' and not (uid==0 and gid==0 and mode=="600"):
                        fout["ueventd.dev.rc"].append('%-25s 0%-5s %-10s %s' % (
                            devname, mode, name2id(uid), name2id(gid)))

                    # init.rc
                    if typ in 'bc' and dev != devname:
                        fout["init.dev.rc"].append("    symlink %-16s %s" %( devname, dev))
                    if typ == 'd':
                        fout["init.dev.rc"].append("    mkdir %s" % (dev))

if __name__ == "__main__":
    parse_dev_txt()

    for fname, body in fout.items():
        with open(fname, "wt") as f:
            print("writing " + fname + "...")
            f.write('\n'.join(body))

