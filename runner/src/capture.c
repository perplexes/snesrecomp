/*
 * capture.c -- generic per-frame capture harness (see capture.h).
 *
 * Self-contained: writes lossless PNG (RGB8) using a minimal encoder that emits
 * a zlib stream made of *uncompressed* DEFLATE blocks (BTYPE=00). That keeps the
 * encoder dependency-free (no zlib/libpng) and trivially correct -- the output
 * is a fully standard PNG, just larger than a compressed one. Frames are large
 * but lossless; the offline assembler re-encodes them to H.264 anyway.
 *
 * Timestamp source: g_sched_total_cycles (engine monotonic master-cycle clock,
 * sched.c) divided by the NTSC master clock (SCHED_MASTER_CLOCK_HZ). This makes
 * each frame's emu_ms track real emulated time, so fast-forward shows up as a
 * larger inter-frame delta rather than being uniformly stretched.
 */
#include "capture.h"
#include "sched.h"            /* g_sched_total_cycles, SCHED_MASTER_CLOCK_HZ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>

#if defined(_WIN32)
#include <direct.h>
#define MKDIR(p) _mkdir(p)
#else
#define MKDIR(p) mkdir(p, 0755)
#endif

extern int snes_frame_counter;   /* snes/snes.h -- defined in snes.c */

/* -- module state ---------------------------------------------------------- */
static int      s_inited   = 0;     /* one-time env probe done */
static int      s_enabled  = 0;     /* RECOMP_CAPTURE_DIR was set */
static char     s_dir[1024];
static long     s_max      = 0;     /* RECOMP_CAPTURE_MAX (0 = unlimited) */
static long     s_index    = 0;     /* present-order frame index */
static FILE    *s_manifest = NULL;
static double   s_wall0_ms = -1.0;  /* host wall baseline */

static double now_wall_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec * 1000.0 + (double)tv.tv_usec / 1000.0;
}

/* -- CRC32 (PNG / standard polynomial) ------------------------------------- */
static uint32_t s_crc_tab[256];
static int s_crc_ready = 0;
static void crc_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (c & 1 ? 0xEDB88320u : 0);
        s_crc_tab[i] = c;
    }
    s_crc_ready = 1;
}

/* -- helpers --------------------------------------------------------------- */
static void put_be32(uint8_t *b, uint32_t v) {
    b[0] = (uint8_t)(v >> 24); b[1] = (uint8_t)(v >> 16);
    b[2] = (uint8_t)(v >> 8);  b[3] = (uint8_t)v;
}

/* Write one PNG chunk: length, type, data, crc(type+data). */
static int png_chunk(FILE *f, const char *type, const uint8_t *data, uint32_t len) {
    uint8_t hdr[8];
    put_be32(hdr, len);
    memcpy(hdr + 4, type, 4);
    if (fwrite(hdr, 1, 8, f) != 8) return -1;
    if (len && fwrite(data, 1, len, f) != len) return -1;
    /* CRC32 over chunk type + data (PNG spec). */
    uint32_t crc = 0xFFFFFFFFu;
    if (!s_crc_ready) crc_init();
    for (int i = 0; i < 4; i++) crc = s_crc_tab[(crc ^ (uint8_t)type[i]) & 0xFF] ^ (crc >> 8);
    for (uint32_t i = 0; i < len; i++) crc = s_crc_tab[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    crc ^= 0xFFFFFFFFu;
    uint8_t cb[4]; put_be32(cb, crc);
    return (fwrite(cb, 1, 4, f) == 4) ? 0 : -1;
}

/* zlib adler32 over the raw (filtered) image bytes. */
static uint32_t adler32(const uint8_t *p, size_t n) {
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < n; i++) { a = (a + p[i]) % 65521u; b = (b + a) % 65521u; }
    return (b << 16) | a;
}

/* Encode RGB8 pixels to a PNG file. `rgb` is height rows of width*3 bytes. */
static int png_write_rgb(const char *path, const uint8_t *rgb, int w, int h) {
    FILE *f = fopen(path, "wb");
    if (!f) return -1;

    static const uint8_t sig[8] = {137,80,78,71,13,10,26,10};
    if (fwrite(sig, 1, 8, f) != 8) { fclose(f); return -1; }

    uint8_t ihdr[13];
    put_be32(ihdr + 0, (uint32_t)w);
    put_be32(ihdr + 4, (uint32_t)h);
    ihdr[8]  = 8;   /* bit depth */
    ihdr[9]  = 2;   /* color type 2 = truecolor RGB */
    ihdr[10] = 0;   /* compression */
    ihdr[11] = 0;   /* filter method */
    ihdr[12] = 0;   /* interlace */
    if (png_chunk(f, "IHDR", ihdr, 13)) { fclose(f); return -1; }

    /* Build the raw filtered stream: each row prefixed with filter byte 0. */
    size_t row = (size_t)w * 3;
    size_t raw_len = (row + 1) * (size_t)h;
    uint8_t *raw = (uint8_t *)malloc(raw_len);
    if (!raw) { fclose(f); return -1; }
    for (int y = 0; y < h; y++) {
        uint8_t *dst = raw + (size_t)y * (row + 1);
        dst[0] = 0;
        memcpy(dst + 1, rgb + (size_t)y * row, row);
    }

    /* zlib stream: 2-byte header + stored DEFLATE blocks + adler32. */
    size_t nblocks = (raw_len + 65534) / 65535;
    if (nblocks == 0) nblocks = 1;
    size_t z_len = 2 + nblocks * 5 + raw_len + 4;
    uint8_t *z = (uint8_t *)malloc(z_len);
    if (!z) { free(raw); fclose(f); return -1; }
    size_t zp = 0;
    z[zp++] = 0x78; z[zp++] = 0x01;        /* CMF/FLG (no preset dict) */
    size_t off = 0;
    while (off < raw_len) {
        size_t chunk = raw_len - off;
        if (chunk > 65535) chunk = 65535;
        int final = (off + chunk >= raw_len) ? 1 : 0;
        z[zp++] = (uint8_t)final;          /* BFINAL, BTYPE=00 */
        z[zp++] = (uint8_t)(chunk & 0xFF);
        z[zp++] = (uint8_t)((chunk >> 8) & 0xFF);
        uint16_t nlen = (uint16_t)(~chunk);
        z[zp++] = (uint8_t)(nlen & 0xFF);
        z[zp++] = (uint8_t)((nlen >> 8) & 0xFF);
        memcpy(z + zp, raw + off, chunk);
        zp += chunk;
        off += chunk;
    }
    uint32_t ad = adler32(raw, raw_len);
    put_be32(z + zp, ad); zp += 4;
    free(raw);

    int rc = png_chunk(f, "IDAT", z, (uint32_t)zp);
    free(z);
    if (rc) { fclose(f); return -1; }
    if (png_chunk(f, "IEND", NULL, 0)) { fclose(f); return -1; }
    fclose(f);
    return 0;
}

/* -- public API ------------------------------------------------------------ */
static void capture_init(void) {
    s_inited = 1;
    const char *dir = getenv("RECOMP_CAPTURE_DIR");
    if (!dir || !*dir) { s_enabled = 0; return; }
    strncpy(s_dir, dir, sizeof(s_dir) - 1);
    s_dir[sizeof(s_dir) - 1] = 0;
    MKDIR(s_dir);
    const char *mx = getenv("RECOMP_CAPTURE_MAX");
    s_max = (mx && *mx) ? strtol(mx, NULL, 10) : 0;

    char mp[1100];
    snprintf(mp, sizeof(mp), "%s/manifest.csv", s_dir);
    s_manifest = fopen(mp, "w");
    if (s_manifest) {
        fprintf(s_manifest, "frame_index,sched_cycles,emu_ms,wall_ms,snes_frame\n");
        fflush(s_manifest);
    }
    s_enabled = 1;
    fprintf(stderr,
        "[capture] armed: dir='%s' max=%ld clock=%.0fHz (emu_ms = cycles/%.3f)\n",
        s_dir, s_max, SCHED_MASTER_CLOCK_HZ, SCHED_MASTER_CLOCK_HZ / 1000.0);
    fflush(stderr);
}

int Capture_Enabled(void) {
    if (!s_inited) capture_init();
    return s_enabled;
}

void Capture_Frame(const uint8_t *pixels, int width, int height, size_t pitch_bytes) {
    if (!s_inited) capture_init();
    if (!s_enabled || !pixels || width <= 0 || height <= 0) return;
    if (s_max > 0 && s_index >= s_max) return;

    /* Snapshot the faithful emulated-time clock BEFORE any I/O. */
    uint64_t cycles = g_sched_total_cycles;
    double emu_ms = (double)cycles * 1000.0 / SCHED_MASTER_CLOCK_HZ;
    if (s_wall0_ms < 0) s_wall0_ms = now_wall_ms();
    double wall_ms = now_wall_ms() - s_wall0_ms;
    int snes_fr = snes_frame_counter;

    /* Repack B,G,R,pad -> R,G,B contiguous. */
    size_t rgb_len = (size_t)width * 3 * (size_t)height;
    uint8_t *rgb = (uint8_t *)malloc(rgb_len);
    if (!rgb) return;
    for (int y = 0; y < height; y++) {
        const uint8_t *src = pixels + (size_t)y * pitch_bytes;
        uint8_t *dst = rgb + (size_t)y * width * 3;
        for (int x = 0; x < width; x++) {
            const uint8_t *p = src + (size_t)x * 4; /* B,G,R,pad */
            dst[x*3 + 0] = p[2]; /* R */
            dst[x*3 + 1] = p[1]; /* G */
            dst[x*3 + 2] = p[0]; /* B */
        }
    }

    char path[1200];
    snprintf(path, sizeof(path), "%s/frame%06ld.png", s_dir, s_index);
    int rc = png_write_rgb(path, rgb, width, height);
    free(rgb);
    if (rc) {
        fprintf(stderr, "[capture] WARN: failed to write %s\n", path);
        return;
    }

    if (s_manifest) {
        fprintf(s_manifest, "%ld,%llu,%.4f,%.3f,%d\n",
                s_index, (unsigned long long)cycles, emu_ms, wall_ms, snes_fr);
        fflush(s_manifest);
    }
    s_index++;
}
