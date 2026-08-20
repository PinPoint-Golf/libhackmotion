/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Mark Liversedge */
#include "hackmotion/device.h"

#include <string.h>
#include <stdio.h>

/* 49535343-fe7d-4ae5-8fa9-9fafd205e455 */
const hm_uuid HM_UUID_TRANSPARENT_UART_SERVICE = {
    {0x49, 0x53, 0x53, 0x43, 0xfe, 0x7d, 0x4a, 0xe5,
     0x8f, 0xa9, 0x9f, 0xaf, 0xd2, 0x05, 0xe4, 0x55}};

/* 413c3893-b7e8-4231-9673-7af7aed06ddc */
const hm_uuid HM_UUID_DATA_CHARACTERISTIC = {
    {0x41, 0x3c, 0x38, 0x93, 0xb7, 0xe8, 0x42, 0x31,
     0x96, 0x73, 0x7a, 0xf7, 0xae, 0xd0, 0x6d, 0xdc}};

/* 1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0  ⛔ never write here */
const hm_uuid HM_UUID_OTA_SERVICE_FORBIDDEN = {
    {0x1d, 0x14, 0xd6, 0xee, 0xfd, 0x63, 0x4f, 0xa1,
     0xbf, 0xa4, 0x8f, 0x47, 0xb4, 0x21, 0x19, 0xf0}};

/* 49535343-1e4d-4bd9-ba61-23c647249616  — accepts writes, never replies */
const hm_uuid HM_UUID_ISSC_PIPE_INERT = {
    {0x49, 0x53, 0x53, 0x43, 0x1e, 0x4d, 0x4b, 0xd9,
     0xba, 0x61, 0x23, 0xc6, 0x47, 0x24, 0x96, 0x16}};

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

hm_status hm_uuid_parse(const char *text, hm_uuid *out)
{
    static const int group_len[5] = {8, 4, 4, 4, 12};
    size_t pos = 0;
    size_t byte = 0;

    if (text == NULL || out == NULL) {
        return HM_ERR_INVALID_ARG;
    }
    if (text[0] == '{') {
        pos = 1;
    }

    for (int g = 0; g < 5; ++g) {
        for (int i = 0; i < group_len[g]; i += 2) {
            int hi = hex_value(text[pos]);
            int lo = (hi < 0) ? -1 : hex_value(text[pos + 1]);
            if (lo < 0) {
                return HM_ERR_INVALID_ARG;
            }
            out->bytes[byte++] = (uint8_t)((hi << 4) | lo);
            pos += 2;
        }
        if (g < 4) {
            if (text[pos] != '-') {
                return HM_ERR_INVALID_ARG;
            }
            pos++;
        }
    }
    if (text[pos] == '}') {
        pos++;
    }
    if (text[pos] != '\0') {
        return HM_ERR_INVALID_ARG;
    }
    return HM_OK;
}

char *hm_uuid_format(const hm_uuid *uuid, char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) {
        return out;
    }
    if (uuid == NULL || out_size < HM_UUID_STRING_SIZE) {
        out[0] = '\0';
        return out;
    }
    (void)snprintf(out, out_size,
                   "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                   uuid->bytes[0], uuid->bytes[1], uuid->bytes[2], uuid->bytes[3],
                   uuid->bytes[4], uuid->bytes[5], uuid->bytes[6], uuid->bytes[7],
                   uuid->bytes[8], uuid->bytes[9], uuid->bytes[10], uuid->bytes[11],
                   uuid->bytes[12], uuid->bytes[13], uuid->bytes[14], uuid->bytes[15]);
    return out;
}

bool hm_uuid_equal(const hm_uuid *a, const hm_uuid *b)
{
    if (a == NULL || b == NULL) {
        return false;
    }
    return memcmp(a->bytes, b->bytes, sizeof(a->bytes)) == 0;
}

bool hm_looks_like_hackmotion(const char *local_name, const hm_uuid *advertised_services,
                              size_t service_count)
{
    /*
     * Name match is sufficient and is the only discriminator many stacks give
     * us: plenty of platforms report no service UUIDs in an advertisement at
     * all.  A service match without the name is accepted too, since a device
     * whose name field was truncated is still worth offering to the user.
     *
     * ⚠ THE PREFIX, NOT THE WHOLE NAME.  "wG3" names a generation, so a later
     * sensor advertises a different string and an exact match would hide it on
     * exactly those stacks that give us nothing else to go on.  The family
     * prefix keeps every generation discoverable; whether one can be SPOKEN to
     * is settled after link-up, by the MTU floor and by the version reply.
     */
    if (local_name != NULL &&
        strncmp(local_name, HM_ADVERTISED_NAME_PREFIX, sizeof(HM_ADVERTISED_NAME_PREFIX) - 1u) ==
            0) {
        return true;
    }
    if (advertised_services != NULL) {
        for (size_t i = 0; i < service_count; ++i) {
            if (hm_uuid_equal(&advertised_services[i], &HM_UUID_TRANSPARENT_UART_SERVICE)) {
                return true;
            }
        }
    }
    return false;
}
