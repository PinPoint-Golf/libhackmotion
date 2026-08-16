/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* Copyright (C) 2026 Mark Liversedge */
/*
 * hm_record.c — the `.hmwire` container.  Chunks in, a versioned file out, and
 * the same chunks back byte-exact.
 *
 * ⚠ Not part of the sans-I/O core.  This file opens files on purpose; it is a
 * separate target and tests/purity.cmake does not look at it.
 *
 * The point of the format is that a recording made today survives a decode fix
 * made later (design §5.6, api-request §2.9).  Two properties follow and both
 * are load-bearing:
 *
 *   - The payload is stored EXACTLY as the transport delivered it.  Nothing is
 *     re-framed, coalesced or normalised on the way through, because the
 *     protocol has no length field, no sequence number and no checksum (§3) and
 *     a boundary lost here cannot be recovered by any later reader.
 *   - The container's own integers are little-endian and SAY SO in the text
 *     header, because §1 warns that byte order is not uniform across this
 *     protocol and a container that left its order to be inferred would be one
 *     more thing to infer.
 */
#include "hackmotion/record.h"

#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------ */
/* Little-endian packing.  Explicit, so the file is identical on any host.    */
/* ------------------------------------------------------------------------ */
static void put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
}

static void put_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
    p[2] = (uint8_t)((v >> 16) & 0xffu);
    p[3] = (uint8_t)((v >> 24) & 0xffu);
}

static void put_i64le(uint8_t *p, int64_t v)
{
    uint64_t u = (uint64_t)v; /* two's complement, well defined on the way in */
    for (int i = 0; i < 8; ++i) {
        p[i] = (uint8_t)((u >> (8 * i)) & 0xffu);
    }
}

static uint32_t get_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static int64_t get_i64le(const uint8_t *p)
{
    uint64_t u = 0;
    for (int i = 7; i >= 0; --i) {
        u = (u << 8) | (uint64_t)p[i];
    }
    return (int64_t)u;
}

/* ------------------------------------------------------------------------ */
/* Defaults                                                                  */
/* ------------------------------------------------------------------------ */
hm_recording_info hm_recording_info_default(void)
{
    hm_recording_info info;
    memset(&info, 0, sizeof(info));
    info.config_bits = HM_CONFIG_OBSERVED_DEFAULT;
    info.layout_version = HM_SAMPLE_LAYOUT_VERSION;
    /* ⚠ A wall clock would be stepped by NTP or DST and corrupt a capture in a
     * way that looks like a sensor fault (types.h).  There is one right answer
     * here and the field records which one the host actually used. */
    memcpy(info.clock, HM_RECORD_CLOCK_MONOTONIC, sizeof(HM_RECORD_CLOCK_MONOTONIC));
    info.identifiers_recorded = false;
    return info;
}

/* ------------------------------------------------------------------------ */
/* Writer                                                                    */
/* ------------------------------------------------------------------------ */
struct hm_recorder {
    FILE     *fp;
    uint64_t  chunks;
    uint64_t  bytes;
    hm_status first_error;
};

/* Header values are one line each, so anything that could end a line early is
 * replaced rather than escaped.  A device id is opaque to us and may come from
 * a platform string we do not control. */
static void write_sanitised(FILE *fp, const char *text, size_t max)
{
    for (size_t i = 0; i < max && text[i] != '\0'; ++i) {
        unsigned char c = (unsigned char)text[i];
        fputc((c < 0x20u || c == 0x7fu) ? '_' : (int)c, fp);
    }
}

hm_status hm_recorder_open(const char *path, const hm_recording_info *info,
                           hm_recorder **out_recorder)
{
    hm_recorder *r;
    hm_recording_info local;

    if (path == NULL || out_recorder == NULL) {
        return HM_ERR_INVALID_ARG;
    }
    *out_recorder = NULL;

    local = (info != NULL) ? *info : hm_recording_info_default();
    if (local.clock[0] == '\0') {
        memcpy(local.clock, HM_RECORD_CLOCK_MONOTONIC, sizeof(HM_RECORD_CLOCK_MONOTONIC));
    }
    if (local.layout_version == 0u) {
        local.layout_version = HM_SAMPLE_LAYOUT_VERSION;
    }

    r = (hm_recorder *)calloc(1, sizeof(*r));
    if (r == NULL) {
        return HM_ERR_NO_MEMORY;
    }

    r->fp = fopen(path, "wb");
    if (r->fp == NULL) {
        free(r);
        return HM_ERR_INVALID_STATE;
    }

    fputs(HM_RECORD_MAGIC "\n", r->fp);
    fputs("device_id=", r->fp);
    write_sanitised(r->fp, local.device_id, sizeof(local.device_id));
    fputc('\n', r->fp);
    if (local.config_legacy) {
        /* The bare `82` start takes no configuration byte at all, so it cannot
         * be spelled as a value of `config` (config.h). */
        fputs("config=legacy\n", r->fp);
    } else {
        fprintf(r->fp, "config=0x%02x\n", (unsigned)local.config_bits);
    }
    fprintf(r->fp, "layout_version=%u\n", (unsigned)local.layout_version);
    fputs("clock=", r->fp);
    write_sanitised(r->fp, local.clock, sizeof(local.clock));
    fputc('\n', r->fp);
    fputs("byte_order=little\n", r->fp);
    fputs(local.identifiers_recorded ? "identifiers=recorded\n" : "identifiers=redacted\n",
          r->fp);
    fputc('\n', r->fp);

    if (ferror(r->fp)) {
        fclose(r->fp);
        free(r);
        return HM_ERR_INVALID_STATE;
    }

    *out_recorder = r;
    return HM_OK;
}

hm_status hm_recorder_write(hm_recorder *recorder, const hm_wire_chunk *chunks, size_t count)
{
    uint8_t header[HM_RECORD_ENTRY_HEADER];

    if (recorder == NULL || (chunks == NULL && count > 0u)) {
        return HM_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < count; ++i) {
        if (chunks[i].length > HM_WIRE_CHUNK_MAX) {
            return HM_ERR_INVALID_ARG;
        }
    }

    for (size_t i = 0; i < count; ++i) {
        const hm_wire_chunk *c = &chunks[i];
        put_u32le(header + 0, (uint32_t)c->length);
        header[4] = c->direction;
        header[5] = c->flags;
        put_u16le(header + 6, 0u);
        put_u32le(header + 8, c->sequence);
        put_i64le(header + 12, c->host_time_us);

        if (fwrite(header, 1, sizeof(header), recorder->fp) != sizeof(header) ||
            (c->length > 0u &&
             fwrite(c->data, 1, c->length, recorder->fp) != (size_t)c->length)) {
            if (recorder->first_error == HM_OK) {
                recorder->first_error = HM_ERR_INVALID_STATE;
            }
            return recorder->first_error;
        }
        recorder->chunks++;
        recorder->bytes += (uint64_t)sizeof(header) + c->length;
    }
    return HM_OK;
}

uint64_t hm_recorder_chunks(const hm_recorder *recorder)
{
    return (recorder != NULL) ? recorder->chunks : 0u;
}

uint64_t hm_recorder_bytes(const hm_recorder *recorder)
{
    return (recorder != NULL) ? recorder->bytes : 0u;
}

hm_status hm_recorder_close(hm_recorder *recorder)
{
    hm_status st;

    if (recorder == NULL) {
        return HM_ERR_INVALID_ARG;
    }
    st = recorder->first_error;
    if (fflush(recorder->fp) != 0 || ferror(recorder->fp)) {
        if (st == HM_OK) {
            st = HM_ERR_INVALID_STATE;
        }
    }
    if (fclose(recorder->fp) != 0 && st == HM_OK) {
        /* ⚠ A capture that filled the disk must not end quietly: the bytes are
         * the artefact and there is no second copy. */
        st = HM_ERR_INVALID_STATE;
    }
    free(recorder);
    return st;
}

/* ------------------------------------------------------------------------ */
/* Reader                                                                    */
/* ------------------------------------------------------------------------ */
struct hm_replay {
    FILE             *fp;
    hm_recording_info info;
    uint64_t          chunks_read;
};

/* Reads one LF-terminated line into `out`, without the LF.  Returns the length,
 * or -1 at end of file, or -2 if the line did not fit. */
static int read_line(FILE *fp, char *out, size_t max)
{
    size_t n = 0;
    int c = fgetc(fp);

    if (c == EOF) {
        return -1;
    }
    while (c != EOF && c != '\n') {
        if (n + 1u >= max) {
            return -2;
        }
        out[n++] = (char)c;
        c = fgetc(fp);
    }
    out[n] = '\0';
    return (int)n;
}

static void copy_field(char *dst, size_t dst_size, const char *src)
{
    size_t n = strlen(src);
    if (n >= dst_size) {
        n = dst_size - 1u;
    }
    memcpy(dst, src, n);
    dst[n] = '\0';
}

hm_status hm_replay_open(const char *path, hm_replay **out_replay)
{
    hm_replay *r;
    char line[HM_RECORD_HEADER_MAX];
    size_t header_bytes = 0;
    int len;

    if (path == NULL || out_replay == NULL) {
        return HM_ERR_INVALID_ARG;
    }
    *out_replay = NULL;

    r = (hm_replay *)calloc(1, sizeof(*r));
    if (r == NULL) {
        return HM_ERR_NO_MEMORY;
    }
    r->info = hm_recording_info_default();

    r->fp = fopen(path, "rb");
    if (r->fp == NULL) {
        free(r);
        return HM_ERR_INVALID_STATE;
    }

    len = read_line(r->fp, line, sizeof(line));
    if (len < 0 || strcmp(line, HM_RECORD_MAGIC) != 0) {
        fclose(r->fp);
        free(r);
        /* ⚠ Wrong magic is malformed, not "an older version".  There is no
         * version 0 in existence and guessing at one would read arbitrary
         * bytes as timestamps. */
        return (len == -2) ? HM_ERR_BUFFER_TOO_SMALL : HM_ERR_MALFORMED;
    }
    header_bytes += (size_t)len + 1u;

    for (;;) {
        char *eq;
        len = read_line(r->fp, line, sizeof(line));
        if (len == -1) {
            fclose(r->fp);
            free(r);
            return HM_ERR_TRUNCATED; /* header never terminated */
        }
        if (len == -2) {
            fclose(r->fp);
            free(r);
            return HM_ERR_BUFFER_TOO_SMALL;
        }
        header_bytes += (size_t)len + 1u;
        if (header_bytes > HM_RECORD_HEADER_MAX) {
            fclose(r->fp);
            free(r);
            return HM_ERR_BUFFER_TOO_SMALL;
        }
        if (len == 0) {
            break; /* blank line ends the header */
        }

        eq = strchr(line, '=');
        if (eq == NULL) {
            continue; /* not a key=value line; ignore, as below */
        }
        *eq = '\0';
        {
            const char *key = line;
            const char *value = eq + 1;

            if (strcmp(key, "device_id") == 0) {
                copy_field(r->info.device_id, sizeof(r->info.device_id), value);
            } else if (strcmp(key, "config") == 0) {
                if (strcmp(value, "legacy") == 0) {
                    r->info.config_legacy = 1u;
                    r->info.config_bits = 0u;
                } else {
                    unsigned bits = 0;
                    if (sscanf(value, "0x%x", &bits) != 1 && sscanf(value, "%u", &bits) != 1) {
                        fclose(r->fp);
                        free(r);
                        return HM_ERR_MALFORMED;
                    }
                    r->info.config_bits = (uint8_t)(bits & 0xffu);
                    r->info.config_legacy = 0u;
                }
            } else if (strcmp(key, "layout_version") == 0) {
                unsigned v = 0;
                if (sscanf(value, "%u", &v) != 1) {
                    fclose(r->fp);
                    free(r);
                    return HM_ERR_MALFORMED;
                }
                r->info.layout_version = v;
            } else if (strcmp(key, "clock") == 0) {
                copy_field(r->info.clock, sizeof(r->info.clock), value);
            } else if (strcmp(key, "byte_order") == 0) {
                if (strcmp(value, "little") != 0) {
                    /* Nothing writes anything else today.  Refusing beats
                     * silently reading every timestamp byte-reversed. */
                    fclose(r->fp);
                    free(r);
                    return HM_ERR_NOT_SUPPORTED;
                }
            } else if (strcmp(key, "identifiers") == 0) {
                r->info.identifiers_recorded = (strcmp(value, "recorded") == 0);
            }
            /* ⚠ Any other key is ignored, exactly as spec §5.1 says to treat an
             * unknown message id.  A recording written by a later version stays
             * readable up to the part this version understands. */
        }
    }

    *out_replay = r;
    return HM_OK;
}

const hm_recording_info *hm_replay_info(const hm_replay *replay)
{
    return (replay != NULL) ? &replay->info : NULL;
}

hm_status hm_replay_next(hm_replay *replay, hm_wire_chunk *out_chunk)
{
    uint8_t header[HM_RECORD_ENTRY_HEADER];
    size_t got;
    uint32_t length;

    if (replay == NULL || out_chunk == NULL) {
        return HM_ERR_INVALID_ARG;
    }

    got = fread(header, 1, sizeof(header), replay->fp);
    if (got == 0u) {
        return feof(replay->fp) ? HM_DONE : HM_ERR_INVALID_STATE;
    }
    if (got != sizeof(header)) {
        return HM_ERR_TRUNCATED;
    }

    length = get_u32le(header + 0);
    if (length > HM_WIRE_CHUNK_MAX) {
        /* ⚠ Refuse, do not clamp.  This is the one place a corrupt or hostile
         * file meets a fixed-size buffer, and a clamp would carry on decoding a
         * file whose framing is already known to be wrong. */
        return HM_ERR_MALFORMED;
    }

    memset(out_chunk, 0, sizeof(*out_chunk));
    out_chunk->length = (uint16_t)length;
    out_chunk->direction = header[4];
    out_chunk->flags = header[5];
    out_chunk->sequence = get_u32le(header + 8);
    out_chunk->host_time_us = get_i64le(header + 12);

    if (length > 0u) {
        if (fread(out_chunk->data, 1, length, replay->fp) != (size_t)length) {
            return HM_ERR_TRUNCATED;
        }
    }
    replay->chunks_read++;
    return HM_OK;
}

uint64_t hm_replay_chunks_read(const hm_replay *replay)
{
    return (replay != NULL) ? replay->chunks_read : 0u;
}

void hm_replay_close(hm_replay *replay)
{
    if (replay == NULL) {
        return;
    }
    fclose(replay->fp);
    free(replay);
}
